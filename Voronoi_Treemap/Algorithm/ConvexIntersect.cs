using System;
using System.Collections.Generic;
using System.Windows;

using Treemap.Voronoi.DataStructures;

namespace Treemap.Voronoi.Algorithm
{
    /// <summary>
    /// Represents the intersect of two convex in 2-D space
    /// </summary>
    class ConvexIntersect
    {
        public Polygon ConvexP { get; set; }
        public Polygon ConvexQ { get; set; }

        /// <summary>
        /// number of ConvexP's vertices 
        /// </summary>
        public int N { get; set; }

        /// <summary>
        /// number of ConvexQ's vertices 
        /// </summary>
        public int M { get; set; }

        private const double Eps = 1E-7;
        enum tInFlag { P_in, Q_in, Unknown };
        enum tcode { Collinear,  No_inters, Proper_inters };

        public Polygon Inters { get; set; }

        /// <summary>
        /// Represents the intersect of two convex in 2-D space
        /// </summary>
        public ConvexIntersect(Polygon _Convex1, Polygon _Convex2)
        {
            this.ConvexP = _Convex1;
            this.ConvexQ = _Convex2;
            this.N = _Convex1.Count;
            this.M = _Convex2.Count;
        }

        private int AreaSign(Vector a, Vector b, Vector c)
        {
            double area = (b.X - a.X) * (c.Y - a.Y) - (c.X - a.X) * (b.Y - a.Y);
            if (area > Eps) return 1;
            else if (area < -Eps) return -1;
            else return 0;
        }

        /// <summary>
        /// Test if c is between a & b horizontally
        /// </summary>
        private static bool Between(Vector a, Vector b, Vector c)
        {
            if (a.X != b.X)
                return (c.X >= a.X && c.X <= b.X) || (c.X <= a.X && c.X >= b.X);
            else
                return (c.Y >= a.Y && c.Y <= b.Y) || (c.Y <= a.Y && c.Y >= b.Y);
        }

        /// <summary>
        /// Test if (a,b) intersect with (c,d)
        /// </summary>
        private tcode Intersect(Vector a, Vector b, Vector c, Vector d, out Vector _inter1, out Vector _inter2)
        {
            double cross = ((d.X - c.X) * (b.Y - a.Y)) - ((b.X - a.X) * (d.Y - c.Y));
            _inter1 = new Vector();
            _inter2 = new Vector();
            if (cross == 0)
            {
                if(AreaSign(a, b, c) == 0)
                {
                    if (Between(a, b, c) && Between(a, b, d))
                    {
                        _inter1 = c;
                        _inter2 = d;
                        return tcode.Collinear;
                    }
                    if (Between(c, d, a) && Between(c, d, b))
                    {
                        _inter1 = a;
                        _inter2 = b;
                        return tcode.Collinear;
                    }
                    if (Between(a, b, c) && Between(c, d, b))
                    {
                        _inter1 = c;
                        _inter2 = b;
                        return tcode.Collinear;
                    }
                    if (Between(a, b, c) && Between(c, d, a))
                    {
                        _inter1 = c;
                        _inter2 = a;
                        return tcode.Collinear;
                    }
                    if (Between(a, b, d) && Between(c, d, b))
                    {
                        _inter1 = d;
                        _inter2 = b;
                        return tcode.Collinear;
                    }
                    if (Between(a, b, d) && Between(c, d, a))
                    {
                        _inter1 = d;
                        _inter2 = a;
                        return tcode.Collinear;
                    }
                }
                return tcode.No_inters;
            }
            double t = (a.X * (d.Y - c.Y) - a.Y * (d.X - c.X) + c.Y * (d.X - c.X) - c.X * (d.Y - c.Y))/cross;
            double s = ((b.X - a.X) * a.Y + c.X * (b.Y - a.Y) - a.X * (b.Y - a.Y) - c.Y * (b.X - a.X))/-cross;
            if(t >= 0 && t <= 1 && s >= 0 && s <= 1)
            {
                _inter1 = new Vector(a.X + t * (b.X - a.X), a.Y + t * (b.Y - a.Y));
                return tcode.Proper_inters;
            }
            return tcode.No_inters;

        }

        /// <summary>
        /// Compute the intersect of two convex in 2-D space
        /// </summary>
        /// <returns>the intersect polygon of two convex in 2-D space</returns>
        public Polygon Compute()
        {
            this.Inters = new Polygon();
            tInFlag inflag = tInFlag.Unknown;
            bool FirstPoint = true;

            int aa = 0; //#advance for a (in P)
            int ba = 0; //#advance for b (in Q)
            int a = 0;  //index on P
            int b = 0;  //index on Q
            int a1;     //prev of a
            int b1;     //prev of b

            Vector A, B; //current directed edges
            Vector origin = new Vector();
            Vector interp_1;
            Vector interp_2;
            
            tcode code;

            do
            {
                if (a == N) a = 0;
                if (b == M) b = 0;

                a1 = (a == 0) ? (N - 1) : (a - 1);
                b1 = (b == 0) ? (M - 1) : (b - 1);

                A = ConvexP[a] - ConvexP[a1];
                B = ConvexQ[b] - ConvexQ[b1];

                int cross = AreaSign(origin, A, B);
                int aHB = AreaSign(ConvexQ[b1], ConvexQ[b], ConvexP[a]);
                int bHA = AreaSign(ConvexP[a1], ConvexP[a], ConvexQ[b]);

                code = Intersect(ConvexP[a1], ConvexP[a], ConvexQ[b1], ConvexQ[b], out interp_1, out interp_2);

                if (code == tcode.Proper_inters)
                {
                    if (inflag == tInFlag.Unknown && FirstPoint)
                    {
                        FirstPoint = false;
                        aa = ba = 0;
                    }
                    Inters.Add(interp_1);
                    if (aHB > 0)
                        inflag = tInFlag.P_in;
                    else if (bHA > 0)
                        inflag = tInFlag.Q_in;

                }
                
                if (code == tcode.Collinear && (A*B) < 0)
                {
                    Inters.Add(interp_1);
                    Inters.Add(interp_2);
                    return Inters;
                }

                if (cross == 0 && aHB < 0 && bHA < 0)
                    return Inters;
                else if (cross == 0 && aHB == 0 && bHA == 0)
                {
                    if (inflag == tInFlag.P_in)
                    {
                        ba++; b++;
                    }
                    else
                    {
                        aa++; a++;
                    }
                }
                else if (cross >= 0)
                {
                    if (bHA > 0)
                    {
                        if (inflag == tInFlag.P_in)
                            Inters.Add(ConvexP[a]);
                        aa++; a++;
                    }
                    else
                    {
                        if (inflag == tInFlag.Q_in)
                            Inters.Add(ConvexQ[b]);
                        ba++; b++;
                    }
                }
                else
                {
                    if (aHB > 0)
                    {
                        if (inflag == tInFlag.Q_in)
                            Inters.Add(ConvexQ[b]);
                        ba++; b++;
                    }
                    else
                    {
                        if (inflag == tInFlag.P_in)
                            Inters.Add(ConvexP[a]);
                        aa++; a++;
                    }
                }

            } while (((aa < N) || (ba < M)) && ((aa < 2 * N) && (ba < 2 * M))); // advance 的次数，只跑一圈
           

            return this.Inters;
        }


    }

}
