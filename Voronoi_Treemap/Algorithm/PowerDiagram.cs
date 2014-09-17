using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using System.Windows;

using Treemap.Voronoi.DataStructures;

namespace Treemap.Voronoi.Algorithm
{
    /// <summary>
    /// Represents the power voronoi diagrom problem
    /// </summary>
    public class PowerDiagram
    {
        public List<Site> Sites { get; set; }

        /// <summary>
        /// the dual solution's convex hull surfaces
        /// </summary>
        public List<TriangularFace> HullFaces { get; set; }

        /// <summary>
        /// the boundary polygon
        /// </summary>
        public Polygon BoundPoly { get; set; }
        private Site InfPoint0 { get; set; }
        private Site InfPoint1 { get; set; }
        private Site InfPoint2 { get; set; }
        private Site InfPoint3 { get; set; }

        private const double Eps = 1E-7;

        /// <summary>
        /// Represents the power voronoi diagrom problem
        /// </summary>
        /// <param name="_Sites">the sites/generators</param>
        /// <param name="_boundPoly">the boundary polygon</param>
        public PowerDiagram(List<Site> _Sites, Polygon _boundPoly)
        {
            SetSites(_Sites);
            SetBound(_boundPoly);
        }

        private void SetSites(List<Site> _Sites)
        {
            this.Sites = _Sites;
            this.HullFaces = null;
        }

        /// <summary>
        /// Set the boundary polygon and the four infinite points
        /// </summary>
        private void SetBound(Polygon _boundPoly)
        {
            this.BoundPoly = _boundPoly;
            double minx = _boundPoly.MinX;
            double miny = _boundPoly.MinY;
            double maxx = _boundPoly.MaxX;
            double maxy = _boundPoly.MaxY;

            InfPoint0 = new Site(2 * minx - maxx, 2 * miny - maxy);
            InfPoint1 = new Site(2 * maxx - minx, 2 * miny - maxy);
            InfPoint2 = new Site(2 * maxx - minx, 2 * maxy - miny);
            InfPoint3 = new Site(2 * minx - maxx, 2 * maxy - miny);

            InfPoint0.SetDummy();
            InfPoint1.SetDummy();
            InfPoint2.SetDummy();
            InfPoint3.SetDummy();

        }

        /// <summary>
        /// Solve the power voronoi diagrom problem
        /// </summary>
        public void Compute()
        {
            List<DualSite> Dual_Sites = new List<DualSite>();
            foreach(Site s in this.Sites)
            {
                Dual_Sites.Add(s.ToDualSite());
            }
            Dual_Sites.Add(InfPoint0.ToDualSite());
            Dual_Sites.Add(InfPoint1.ToDualSite());
            Dual_Sites.Add(InfPoint2.ToDualSite());
            Dual_Sites.Add(InfPoint3.ToDualSite());
            ConvexHull hull = new ConvexHull(Dual_Sites);
            HullFaces = hull.Compute();

            ComputeData();
        }

        private List<TriangularFace> GetNeighborface(Edge e)
        {
            List<TriangularFace> result = new List<TriangularFace>();
            Site s = e.Vertex0.OriginalSite;
            Edge now = e;
            do
            {
                s.NeighborSites.Add(now.Vertex1.OriginalSite);
                result.Add(now.NeighborFace);
                now = now.NeighborFace.GetEdge(now.Vertex0, now.Vertex1).NextEdge;
            } while (now != e);

            return result;
        }

        private void ComputeData()
        {
            foreach (TriangularFace f in HullFaces)
            {
                if (f.Normal.Z < -Eps)
                {
                    List<Edge> edges = new List<Edge>();
                    edges.Add(f.Edge0);
                    edges.Add(f.Edge1);
                    edges.Add(f.Edge2);

                    foreach (Edge e in edges)
                    {
                        DualSite dest = e.Vertex0;
                        if (!dest.Visited)
                        {
                            dest.OriginalSite.NeighborSites = new List<Site>();
                            Site site = dest.OriginalSite;
                            dest.Visited = true;
                            if (site.IsDummy())
                            {
                                continue;
                            }

                            List<TriangularFace> tmp_faces = GetNeighborface(e);
                            site.Polyon = new Polygon();

						    foreach (TriangularFace face in tmp_faces) 
                            {
							    Vector point = face.GetDualPoint();
                                site.Polyon.Add(point);
						    }
                            
                            if (!site.IsDummy())
                            {
                                //@site.polyon
                                //@BoundPoly
                                ConvexIntersect clippoly = new ConvexIntersect(site.Polyon, BoundPoly);
                                site.ClipPolyon = clippoly.Compute();
                                if (site.ClipPolyon.Count == 0)
                                    site.ClipPolyon = site.Polyon;
                            }

                        }
                    }
                }
            }


        }

    }

}
