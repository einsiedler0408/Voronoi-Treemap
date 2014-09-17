using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

using Treemap.Voronoi.DataStructures;


namespace Treemap.Voronoi.Algorithm
{
    /// <summary>
    /// Represents a single layer Voronoi Treemap
    /// </summary>
    class VoronoiTreemapSingleLayer
    {
        public Polygon Bound { get; set; }
        public List<double> Attribute { get; set; }
        public List<Site> Sites { get; set; }

        public double Error { get; set; }
        public int NumIter { get; set; }

        /// <summary>
        /// Sum of the attributes
        /// </summary>
        private double SumAttr { get; set; }
        
        private const double Eps = 1;

        /// <summary>
        /// error threshold (default 0.2)
        /// </summary>
        private double EThreshold { get; set; }
        private int MaxIter { get; set; }
        private double CurrentMinNegativeWeight { get; set; }

        /// <summary>
        /// Represents a single layer Voronoi Treemap
        /// </summary>
        public VoronoiTreemapSingleLayer(List<double> _attribute, Polygon _bound, double _e_threshold = 1E-2, int _max_iter = 500)
        {
            this.Attribute = _attribute;
            this.Bound = _bound;
            this.EThreshold = _e_threshold;
            this.MaxIter = _max_iter;
            SetSites();
        }

        /// <summary>
        /// Recursively set the sites' locations
        /// </summary>
        private void SetSites()
        {
            int count = Attribute.Count;
            Int32[] indices = Enumerable.Range(0, count).ToArray();
            Array.Sort(Attribute.ToArray(), indices);
            SumAttr = Attribute.Sum();
            Sites = new List<Site>(Attribute.Capacity);
            for (int i = 0; i < Attribute.Count; i++)
                Sites.Add(null);

            Random rand = new Random();

            double[] attrsorted = new double[count];
            for (int i = 0; i < count; i++)
                attrsorted[i] = Attribute[indices[i]];

            double[] tmp_boundrect = new double[4];
            tmp_boundrect[0] = Bound.MinX;
            tmp_boundrect[1] = Bound.MaxX;
            tmp_boundrect[2] = Bound.MinY;
            tmp_boundrect[3] = Bound.MaxY;

            SetSiteProcess(rand, indices, attrsorted, tmp_boundrect);
        }

        private void SetSiteProcess(Random rand, Int32[] indices, double[] unseted, double[] boundrect)
        {
            int count = unseted.Count<double>();
            double x, y;
            Vector p;
            do
            {
                x = boundrect[0] + (boundrect[1] - boundrect[0]) * rand.NextDouble();
                y = boundrect[2] + (boundrect[3] - boundrect[2]) * rand.NextDouble();
                p = new Vector(x, y);
            } while (!Bound.IsInPolygon(p));
            Sites[indices[count - 1]] = new Site(x, y, unseted[count - 1], Eps);

            if (count == 1)
            {
                return;
            }
            double maxattr = 0.3 * unseted[count - 1];
            double sumattr = unseted[0];
            int start = 0;
            for (int i = 1; i < count; i++)
            {
                sumattr += unseted[i];
                if (sumattr > maxattr || i == count-1)
                {
                    int length = i - start;
                    Int32[] tmp_indices = new Int32[length];
                    double[] tmp_unseted = new double[length];
                    for (int j = 0; j < length; j++)
                    {
                        tmp_indices[j] = indices[start + j];
                        tmp_unseted[j] = unseted[start + j];
                    }
                    //double x, y;
                    //Vector p;
                    do
                    {
                        x = boundrect[0] + (boundrect[1] - boundrect[0]) * rand.NextDouble();
                        y = boundrect[2] + (boundrect[3] - boundrect[2]) * rand.NextDouble();
                        p = new Vector(x, y);
                    } while (!Bound.IsInPolygon(p));
                    double arearoothalf = Math.Sqrt(sumattr / SumAttr * Bound.GetArea())/2;
                    double[] tmp_boundrect = new double[4];
                    tmp_boundrect[0] = x - arearoothalf;
                    tmp_boundrect[1] = x + arearoothalf;
                    tmp_boundrect[2] = y - arearoothalf;
                    tmp_boundrect[3] = y + arearoothalf;

                    SetSiteProcess(rand, tmp_indices, tmp_unseted, tmp_boundrect);

                    start = i;
                    sumattr = unseted[i];
                }
            }
        }

        public void AdaptPositionsWeights(List<Site> Sites)
        {
            foreach(Site s in Sites)
            {
                s.Location = s.ClipPolyon.GetCentroid();
                s.Weight = Math.Max(s.Weight, Eps);
            }

        }


        private void AdaptWeights(List<Site> Sites)
        {
            CurrentMinNegativeWeight = 0;
            foreach (Site s in Sites)
            {
                double area_current = s.ClipPolyon.GetArea();
                double area_target = s.Attribute / SumAttr * Bound.GetArea();

                double radius_current = Math.Sqrt(area_current / Math.PI);
                double radius_target = Math.Sqrt(area_target / Math.PI);
                double deltaCircle = radius_current - radius_target;
                double f_adapt = (area_target / area_current);

                
                if ((f_adapt > 1.1 && s.LastF < 0.9) || (f_adapt < 0.9 && s.LastF > 1.1))
                    f_adapt = Math.Sqrt(f_adapt);
                if (f_adapt < 1.1 && f_adapt > 0.9 && s.Weight != 1)
                    f_adapt = Math.Sqrt(f_adapt);

                if (s.Weight < 10)
                    f_adapt = f_adapt * f_adapt;
                if (s.Weight > 10)
                    f_adapt = Math.Sqrt(f_adapt);

                s.LastF = f_adapt;
                s.Weight *= f_adapt;

                if (s.Weight < Eps)
                {
                    double radius_new2 = Math.Sqrt(s.Weight) - deltaCircle;
                    if (radius_new2 < 0)
                    {
                        s.Weight = -(radius_new2 * radius_new2);
                        if (s.Weight < CurrentMinNegativeWeight)
                            CurrentMinNegativeWeight = s.Weight;
                    }

                }
            }

            if (CurrentMinNegativeWeight < 0)
            {
                CurrentMinNegativeWeight = -CurrentMinNegativeWeight;
                foreach (Site s in Sites)
                {
                    s.Weight += CurrentMinNegativeWeight + Eps;
                }
            }

            FixWeights();
        }

        private void FixWeights()
        {
            double fmin = 1;
            foreach (Site s in Sites)
            {
                List<Site> neigh = s.NeighborSites;
                foreach (Site t in neigh)
                {
                    Vector tmp = s.Location - t.Location;
                    double f = tmp.LengthSquared / (Math.Abs(s.Weight - t.Weight) + Eps);
                    if (f < fmin)
                        fmin = f;
                }
            }
            foreach (Site s in Sites)
            {
                s.Weight *= fmin;
            }
        }


        public double GetError(List<Site> Sites)
        {
            double Error = 0;
            foreach (Site s in Sites)
            {
                double area_current = s.ClipPolyon.GetArea();
                double area_target = s.Attribute / SumAttr * Bound.GetArea();
                Error = Math.Max(Math.Abs(area_current - area_target)/area_target, Error);
                //Error = Math.Max(Math.Abs(area_current - area_target) / Bound.get_Area(), Error);
            }
            return Error;
        }

        public List<Site> Compute()
        {
            PowerDiagram pd = new PowerDiagram(Sites, Bound);
            pd.Compute();

            for (int i = 1; i <= MaxIter; i++)
            {
                AdaptPositionsWeights(Sites);
                AdaptWeights(Sites);
                pd.Compute();

                Error = GetError(Sites);

                if (Error < EThreshold)
                {
                    NumIter = i;
                    break;
                }
            }
            return Sites;
        }

    }

}
