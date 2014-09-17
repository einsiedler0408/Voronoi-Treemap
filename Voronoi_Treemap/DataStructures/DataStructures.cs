using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using System.Windows;

namespace Treemap.Voronoi.DataStructures
{
    /// <summary>
    /// Repersents a triangular face in 3-D space (vertices/edges in counter-clockwise order) 
    /// </summary>
    public class TriangularFace
    {
        public DualSite Vertex0 { get; set; }
        public DualSite Vertex1 { get; set; }
        public DualSite Vertex2 { get; set; }

        /// <summary>
        /// Repersents the unit normal vector (pointing outwards)
        /// </summary>
        
        public Vector3D Normal { get; set; }

        public Edge Edge0 { get; set; }
        public Edge Edge1 { get; set; }
        public Edge Edge2 { get; set; }

        /// <summary>
        /// Repersents the dual point (with z = 0) of the face
        /// </summary>
        public Vector DualPoint { get; set; }

        private const double Eps = 1e-7;

        /// <summary>
        /// If it's a surface of the convex hull
        /// </summary>
        public bool Valid { get; set; }

        /// <summary>
        /// Repersents a triangular face in 3-D space 
        /// </summary>
        /// <param name="v0">vertices in counter-clockwise order</param>
        /// <param name="v1"></param>
        /// <param name="v2"></param>
        public TriangularFace(DualSite v0, DualSite v1, DualSite v2)
        {
            this.Vertex0 = v0;
            this.Vertex1 = v1;
            this.Vertex2 = v2;
            this.Edge0 = new Edge(v0, v1);
            this.Edge1 = new Edge(v1, v2);
            this.Edge2 = new Edge(v2, v0);

            this.Edge0.NextEdge = this.Edge1;
            this.Edge1.NextEdge = this.Edge2;
            this.Edge2.NextEdge = this.Edge0;

            this.Normal = Vector3D.CrossProduct(v1.Coordinate - v0.Coordinate, v2.Coordinate - v0.Coordinate);
            this.Normal.Normalize();
            this.DualPoint = new Vector();
            this.Valid = true;
        }

        /// <summary>
        /// get the dual point in 2-D space(z = 0) of this face
        /// </summary>
        /// <returns>the dual point in 2-D space(z = 0)</returns>
        public Vector GetDualPoint()
        {
            if (DualPoint.X == 0 && DualPoint.Y == 0)
            {
                Vector3D p1 = Vertex0.Coordinate;
                Vector3D p2 = Vertex1.Coordinate;
                Vector3D p3 = Vertex2.Coordinate;
                double a = p1.Y * (p2.Z - p3.Z) + p2.Y * (p3.Z - p1.Z) + p3.Y * (p1.Z - p2.Z);
                double b = p1.Z * (p2.X - p3.X) + p2.Z * (p3.X - p1.X) + p3.Z * (p1.X - p2.X);
                double c = -0.5 / (p1.X * (p2.Y - p3.Y) + p2.X * (p3.Y - p1.Y) + p3.X * (p1.Y - p2.Y));
                //double d = -1 * (p1.X * (p2.Y * p3.Z - p3.Y * p2.Z) + p2.X * (p3.Y * p1.Z - p1.Y * p3.Z) + p3.X * (p1.Y * p2.Z - p2.Y * p1.Z));
                DualPoint = new Vector(a * c, b * c);
            }
            return DualPoint;
        }

        /// <summary>
        /// If v is outside of the face (refer to normal vector) 
        /// </summary>
        /// <param name="v">he vertex to be tested</param>
        /// <returns>True if v is outside of the face (refer to normal vector) </returns>
        public bool VOutFace(DualSite v)
        {
            if (Vector3D.DotProduct(Normal, v.Coordinate - Vertex0.Coordinate) > Eps)
                return true;
            else
                return false;
        }

        /// <summary>
        /// Set the neighboring triangular faces (convex hull)
        /// </summary>
        /// <param name="neighbor">the neighboring triangular face</param>
        /// <param name="v1">a vertex of the shared edge</param>
        /// <param name="v2">the other vertex</param>
        public void SetNeighbor(TriangularFace neighbor, DualSite v1, DualSite v2)
        {
            Edge edge1 = this.GetEdge(v1, v2);
            if (edge1 != null)
            {
                edge1.SetFace(neighbor);
                Edge edge2 = neighbor.GetEdge(v1, v2);
                edge2.SetFace(this);
            }
        }

        /// <summary>
        /// Get the edge of this face with vertices v1, v2
        /// </summary>
        /// <param name="v1">a vertex of the edge</param>
        /// <param name="v2">the other vertex</param>
        /// <returns>null if it cannot be found</returns>
        public Edge GetEdge(DualSite v1, DualSite v2)
        {
            if (Edge0.EqualEdge(v1, v2))
                return Edge0;
            else if (Edge1.EqualEdge(v1, v2))
                return Edge1;
            else if (Edge2.EqualEdge(v1, v2))
                return Edge2;
            else
                return null;
        }

        public override string ToString()
        {
            return String.Format("({0}, {1}, {2})", this.Vertex0, this.Vertex1, this.Vertex2);
        }

    }

    /// <summary>
    /// Repersents a edge (only three edges) of the triangular face
    /// </summary>
    public class Edge
    {
        public DualSite Vertex0 { get; set; }
        public DualSite Vertex1 { get; set; }

        /// <summary>
        /// Repersents the neighboring face which has the same edge (this edge) 
        /// </summary>
        public TriangularFace NeighborFace { get; set; }

        /// <summary>
        /// Repersents the next edge in conter-clockwise order
        /// </summary>
        public Edge NextEdge { get; set; }
        public Edge(DualSite v0, DualSite v1)
        {
            this.Vertex0 = v0;
            this.Vertex1 = v1;
        }
        /// <summary>
        /// Set the neighboring face which has the same edge (this edge) 
        /// </summary>
        /// <param name="f">the neighboring face</param>
        public void SetFace(TriangularFace f)
        {
            this.NeighborFace = f;
        }

        /// <summary>
        /// If this edge is the edge(v0, v1)
        /// </summary>
        /// <param name="v0"></param>
        /// <param name="v1"></param>
        /// <returns>True if this edge's vertices are v0 & v1</returns>
        public bool EqualEdge(DualSite v0, DualSite v1)
        {
            if ((Vertex0 == v0 && Vertex1 == v1) || (Vertex1 == v0 && Vertex0 == v1))
                return true;
            else
                return false;
        }
    }

    /// <summary>
    /// Represents the dual point of a site/generator in 3-D space
    /// </summary>
    public class DualSite
    {
        public Vector3D Coordinate { get; set; }

        /// <summary>
        /// Represents the original site/generator
        /// </summary>
        public Site OriginalSite { get; set; }

        /// <summary>
        /// if it's visited when handling the dual solution of a power diagram
        /// </summary>
        public bool Visited { get; set; }

        /// <summary>
        /// Represents the dual point of a site/generator in 3-D space
        /// </summary>
        public DualSite() : this(0, 0, 0) { }

        /// <summary>
        /// Represents the dual point of a site/generator in 3-D space
        /// </summary>
        public DualSite(double _x, double _y, double _z)
        {
            this.Coordinate = new Vector3D(_x, _y, _z);
            this.Visited = false;
        }

        /// <summary>
        /// Represents the dual point of a site/generator in 3-D space
        /// </summary>
        public DualSite(double _x, double _y, double _z, Site _originalSite)
            : this(_x, _y, _z)
        {
            this.OriginalSite = _originalSite;
        }

    }

    /// <summary>
    /// Represents a site/generator in voronoi treemap
    /// </summary>
    public class Site
    {
        /// <summary>
        /// Represents the location in 2-D space
        /// </summary>
        public Vector Location { get; set; }
        public double Weight { get; set; }
        public double Attribute { get; set; }

        /// <summary>
        /// Represents the last factor in treemap iterations
        /// </summary>
        public double LastF { get; set; }

        /// <summary>
        /// Represents the neighbors that share same edges
        /// </summary>
        public List<Site> NeighborSites { get; set; }

        /// <summary>
        /// Represents the voronoi cell without clipped by the _boundary
        /// </summary>
        public Polygon Polyon { get; set; }

        /// <summary>
        /// Represents the voronoi cell clipped by the _boundary
        /// </summary>
        public Polygon ClipPolyon { get; set; }

        /// <summary>
        /// If the site/generator is an infinite point
        /// </summary>
        private bool Dummy { get; set; }
        private const double NearlyZero = 1E-10;

        /// <summary>
        /// Represents a site/generator in voronoi treemap
        /// </summary>
        public Site(double _x, double _y)
        {
            this.Location = new Vector(_x, _y);
            this.Weight = NearlyZero;
            this.Dummy = false;
            this.LastF = 1;
        }

        /// <summary>
        /// Represents a site/generator in voronoi treemap
        /// </summary>
        public Site(double _x, double _y, double _attribute, double _weight) : this(_x, _y)
        {
            this.Attribute = _attribute;
            this.Weight = _weight;
        }

        public void SetDummy()
        {
            this.Dummy = true;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>True if the site/generator is an infinite point </returns>
        public bool IsDummy()
        {
            return Dummy;
        }

        /// <summary>
        /// Represents the dual point in 3-D space
        /// </summary>
        /// <returns>the dual point in 3-D space of this site/generator</returns>
        public DualSite ToDualSite()
        {
            return new DualSite(Location.X, Location.Y, Location.X * Location.X + Location.Y * Location.Y - Weight, this);
        }

    }

    /// <summary>
    /// Represents a polygon (vertices in counter-clockwise order)
    /// </summary>
    public class Polygon
    {
        /// <summary>
        /// vertices in counter-clockwise order
        /// </summary>
        public List<Vector> Vertices { get; set; }
        private double Area { get; set; }
        private Vector Centroid { get; set; }
        private bool CentroidSeted { get; set; }

        private const double Eps = 1e-7;
        public double MinX
        {
            get { return Vertices.Min(point => point.X); }
        }

        public double MinY
        {
            get { return Vertices.Min(point => point.Y); }
        }
            
        public double MaxX
        {
            get { return Vertices.Max(point => point.X); }
        }
            
        public double MaxY
        {
            get { return Vertices.Max(point => point.Y); }
        }
        
        public Polygon() : this(new List<Vector>()) { }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="_vertices">vertices in counter-clockwise order</param>
        public Polygon(List<Vector> _vertices)
        {
            this.Vertices = _vertices;
            this.Area = 0;
            this.CentroidSeted = false;
        }

        /// <summary>
        /// Add a new vertex to this polygon
        /// </summary>
        /// <param name="p">vertex (in counter-clockwise order)</param>
        public void Add(Vector p)
        {
            Vertices.Add(p);
        }

        /// <summary>
        /// the number of vertices
        /// </summary>
        public int Count 
        {
            get { return Vertices.Count; }
        }

        /// <returns> the index'th vertex (in counter-clockwise order) </returns>
        public Vector this[int index]
        {
            get { return Vertices[index]; }
            set { Vertices[index] = value; }
        }

        /// <summary>
        /// Get the area size of this polygon
        /// </summary>
        /// <returns>the area size of this polygon</returns>
        public double GetArea()
        {
            if (this.Area > 0)
                return this.Area;

            double area = 0;

            int attribute = this.Count - 1;
            for (int i = 0; i < attribute; i++)
            {
                area += Vertices[i].X * Vertices[i + 1].Y - Vertices[i + 1].X * Vertices[i].Y;
            }
            area += Vertices[attribute].X * Vertices[0].Y - Vertices[0].X * Vertices[attribute].Y;
            this.Area = Math.Abs(area) * 0.5;
            return this.Area;
        }

        /// <summary>
        /// Get the centroid of this polygon
        /// </summary>
        /// <returns>the centroid of this polygon</returns>
        public Vector GetCentroid()
        {
            if (CentroidSeted)
                return Centroid;
            double x = 0;
            double y = 0;
            double area;
            double denominator = GetArea() * 6;

            int attribute = this.Count - 1;
            for (int i = 0; i < attribute; i++)
            {
                area = Vertices[i].X * Vertices[i + 1].Y - Vertices[i + 1].X * Vertices[i].Y;
                x += (Vertices[i].X + Vertices[i + 1].X) * area;
                y += (Vertices[i].Y + Vertices[i + 1].Y) * area;
            }
            area = Vertices[attribute].X * Vertices[0].Y - Vertices[0].X * Vertices[attribute].Y;
            x += (Vertices[attribute].X + Vertices[0].X) * area;
            y += (Vertices[attribute].Y + Vertices[0].Y) * area;

            this.Centroid = new Vector(Math.Abs(x / denominator), Math.Abs(y / denominator));
            this.CentroidSeted = true;
            return this.Centroid;
        }

        /// <summary>
        /// If the vertex is in this polygon
        /// </summary>
        /// <param name="p">the vertex to be tested</param>
        /// <returns>True if the vertex is in this polygon</returns>
        public bool IsInPolygon(Vector p)
        {
            int attribute = this.Count - 1;
            double tmp = Vector.CrossProduct(Vertices[0] - Vertices[attribute], p - Vertices[attribute]);
            if (tmp < -Eps) 
                return false;
            for (int i = 0; i < attribute; i++)
            {
                tmp = Vector.CrossProduct(Vertices[i + 1] - Vertices[i], p - Vertices[i]);
                if (tmp < -Eps)
                    return false;
            }
            return true;
        }

        /// <summary>
        /// return the List(v0.x, v0.y, v1.x, v1.y, ...)
        /// </summary>
        /// <returns>the List(v0.x, v0.y, v1.x, v1.y, ...)</returns>
        public List<double> ToListDouble()
        {
            List<double> result = new List<double>();
            foreach (Vector p in this.Vertices)
            {
                result.Add(p.X);
                result.Add(p.Y);
            }
            return result;
        }



    }

    public class TreeNode<T>
    {
        public T Attribute { get; set; }
        public List<TreeNode<T>> Children { get; set; }

        public TreeNode(T _Attribute)
        {
            this.Attribute = _Attribute;
            this.Children = new List<TreeNode<T>>();
        }
        public void AddChild(TreeNode<T> child)
        {
            Children.Add(child);
        }

    }

    public class Tree<T>
    {
        public TreeNode<T> Root { get; set; }
        public Tree(T _Attribute)
        {
            this.Root = new TreeNode<T>(_Attribute);
        }
    }

}
