using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using System.Windows;

using Treemap.Voronoi.DataStructures;

namespace Treemap.Voronoi.Algorithm
{
    /// <summary>
    /// Represents the convex hull in 3-D space
    /// </summary>
    public class ConvexHull
    {
        /// <summary>
        /// vertices needed to compute the convex hull in 3-D space
        /// </summary>
        public List<DualSite> Vertices { get; set; }

        /// <summary>
        /// number of the vertices
        /// </summary>
        public int NumVertex { get; set; }

        /// <summary>
        /// surfaces of this convex hull in 3-D space
        /// </summary>
        public List<TriangularFace> HullFace { get; set; }
        public int NumFace { get; set; }

        private const double Eps = 1e-7;

        /// <summary>
        /// the horizon edges found in a DFS
        /// </summary>
        private List<Edge> Horizon { get; set; }

        /// <summary>
        /// Represents the convex hull in 3-D space
        /// </summary>
        /// <param name="_vertices">vertices in any order</param>
        public ConvexHull(List<DualSite> _vertices)
        {
            this.Vertices = _vertices;
            this.NumVertex = _vertices.Count;
            this.HullFace = new List<TriangularFace>();
            this.Horizon = new List<Edge>();
            this.NumFace = 0;
        }

        /// <summary>
        /// let the first three vertices noncollinear and the first four vertices noncoplanar
        /// </summary>
        private void Prepare()
        {
            Vector3D tmp_vector = new Vector3D();
            for (int i = 2; i < NumVertex; i++)
            {
                tmp_vector = Vector3D.CrossProduct(Vertices[1].Coordinate - Vertices[0].Coordinate, Vertices[i].Coordinate - Vertices[0].Coordinate);
                if (tmp_vector.LengthSquared > Eps)//三点不共线
                {
                    if (i != 2)
                    {
                        DualSite tmp = Vertices[i];
                        Vertices[i] = Vertices[2];
                        Vertices[2] = tmp;
                    }
                    break;
                }
            }

            for (int i = 3; i < NumVertex; i++)
            {
                if (Math.Abs(Vector3D.DotProduct(tmp_vector, Vertices[i].Coordinate - Vertices[0].Coordinate)) > Eps)//四点不共面
                {
                    if (i != 3)
                    {
                        DualSite tmp = Vertices[i];
                        Vertices[i] = Vertices[3];
                        Vertices[3] = tmp;
                    }
                    break;
                }
            }

            //为使面法向量方向指向凸包外
            TriangularFace tmp_face = new TriangularFace(Vertices[0], Vertices[1], Vertices[2]);
            if (tmp_face.VOutFace(Vertices[3]))
            {
                DualSite tmp = Vertices[2];
                Vertices[2] = Vertices[3];
                Vertices[3] = tmp;
            }
            HullFace.Add(new TriangularFace(Vertices[0], Vertices[1], Vertices[2]));
            HullFace.Add(new TriangularFace(Vertices[1], Vertices[3], Vertices[2]));
            HullFace.Add(new TriangularFace(Vertices[0], Vertices[2], Vertices[3]));
            HullFace.Add(new TriangularFace(Vertices[0], Vertices[3], Vertices[1]));

            HullFace[0].SetNeighbor(HullFace[1], Vertices[1], Vertices[2]);
            HullFace[0].SetNeighbor(HullFace[2], Vertices[0], Vertices[2]);
            HullFace[0].SetNeighbor(HullFace[3], Vertices[0], Vertices[1]);
            HullFace[1].SetNeighbor(HullFace[2], Vertices[2], Vertices[3]);
            HullFace[1].SetNeighbor(HullFace[3], Vertices[1], Vertices[3]);
            HullFace[2].SetNeighbor(HullFace[3], Vertices[0], Vertices[3]);

            NumFace = 4;

        }

        /// <summary>
        /// using a DFS to find the horizon
        /// </summary>
        /// <param name="f">root of the DFS</param>
        /// <param name="v">the vertex to be computed</param>
        private void ComputeHorizon(TriangularFace f, DualSite v)
        {
            f.Valid = false;
            DFSProcess(f.Edge0, v);
            DFSProcess(f.Edge1, v);
            DFSProcess(f.Edge2, v);
        }

        private void DFSProcess(Edge e, DualSite v)
        {
            TriangularFace f = e.NeighborFace;
            if (f.Valid)
            {
                if (f.VOutFace(v))
                    ComputeHorizon(f, v);
                else
                    Horizon.Add(e);

            }
        }

        /// <summary>
        /// sort the horizons in counter-clockwise order (tail-to-head)
        /// </summary>
        private void SortHorizons()
        {
            for (int ii = 0; ii < Horizon.Count - 1; ii++)
            {
                for (int jj = ii + 1; jj < Horizon.Count; jj++)
                {
                    if (Horizon[jj].Vertex0 == Horizon[ii].Vertex1)
                    {
                        if (jj != ii + 1)
                        {
                            Edge tmp_e = Horizon[jj];
                            Horizon[jj] = Horizon[ii + 1];
                            Horizon[ii + 1] = tmp_e;
                        }
                        break;
                    }
                }
            }
        }

        private bool IsHorizonSorted()
        {
            for (int ii = 0; ii < Horizon.Count - 1; ii++)
            {
                if (Horizon[ii].Vertex1 != Horizon[ii + 1].Vertex0)
                    return false;
            }
            if (Horizon[Horizon.Count-1].Vertex1 != Horizon[0].Vertex0)
                return false;
            return true;
        }

        /// <summary>
        /// Compute the convex hull in 3-D space
        /// </summary>
        /// <returns>surfaces of the convex hull</returns>
        public List<TriangularFace> Compute()
        {
            NumFace = 0;
            if (NumVertex < 4)
                return null;

            Prepare();

            for (int i = 4; i < NumVertex; i++)
                for (int j = 0; j < NumFace; j++)
                {
                    if (HullFace[j].Valid && HullFace[j].VOutFace(Vertices[i]))
                    {
                        Horizon.Clear();
                        ComputeHorizon(HullFace[j], Vertices[i]);
                        SortHorizons();
                    
                        TriangularFace first = null;
                        TriangularFace last = null;
                        DualSite v = Vertices[i];
                        for (int ii = 0; ii < Horizon.Count; ii++)
                        {
                            Edge e = Horizon[ii];
                            TriangularFace f = e.NeighborFace;
                            TriangularFace add = new TriangularFace(e.Vertex0, e.Vertex1, v);
                            HullFace.Add(add);
                            NumFace++;
                            f.SetNeighbor(add, e.Vertex0, e.Vertex1);
                            if (first == null)
                                first = add;
                            else
                            {
                                add.SetNeighbor(last, e.Vertex0, v);
                            }
                            last = add;
                        }

                        last.SetNeighbor(first, last.Edge0.Vertex1, Vertices[i]);
                        break;
                    }
                }

            List<TriangularFace> result = new List<TriangularFace>();
            for (int i = 0; i < HullFace.Count; i++)
            {
                if (HullFace[i].Valid)
                    result.Add(HullFace[i]);
            }
            return result;
        }


    }

}
