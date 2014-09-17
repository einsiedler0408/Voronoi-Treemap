using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Windows;

using Treemap.Voronoi.Algorithm;
using Treemap.Voronoi.DataStructures;


namespace Treemap.Voronoi
{
    /// <summary>
    /// Represents the Voronoi Treemap Problem
    /// </summary>
    public class VoronoiTreemap
    {
        public Tree<double> AttrTree;
        public int MaxIter;
        public double EThreshold;
        public List<double> Boundary;

        public Tree<Polygon> Treemap;

        private double _MaxError;
        public double MaxError
        {
            get { return _MaxError; }
        }


        private int _ActualIter;
        public int ActualIter
        {
            get { return _ActualIter; }
        }

        //@param in ITree AttrTree
        //@param in int MaxIter
        //@param in double EThreshold
        //@param in List<Point2D> Boundary
        //@param out ITree Treemap
        public VoronoiTreemap(Tree<double> _tree, List<double> _boundary, double _eThreshold = 1E-2, int _maxiter = 500)
        {
            this.AttrTree = _tree;
            this.Boundary = _boundary;
            this.EThreshold = _eThreshold;
            this.MaxIter = _maxiter;
        }

        /// <summary>
        /// Recursively compute the voronoi treemap (using a single layer problem)
        /// </summary>
        public Tree<Polygon> Compute()
        {
            this.Treemap = new Tree<Polygon>(ToPolygon(Boundary));
            this._MaxError = 0;
            this._ActualIter = 0;
            ComputeProcess(AttrTree.Root, Treemap.Root);

            return this.Treemap;
        }

        public static Polygon ToPolygon(List<double> p)
        {
            Polygon t = new Polygon();
            for (int i = 0; i < p.Count; i = i + 2)
            {
                t.Add(new Vector(p[i], p[i + 1]));
            }
            return t;
        }

        /// <summary>
        /// Recursively compute the voronoi treemap (using a single layer problem)
        /// </summary>
        /// <param name="treenode">the node of the tree</param>
        /// <param name="treemapNode">the node of the treemap</param>
        private void ComputeProcess(TreeNode<double> treenode, TreeNode<Polygon> treemapNode)
        {
            var treeChildren = treenode.Children;
            int count = treeChildren.Count;

            if (count == 0)
            {
                return;
            }

            if (count == 1)
            {
                TreeNode<Polygon> node = new TreeNode<Polygon>(treemapNode.Attribute);
                treemapNode.AddChild(node);
                ComputeProcess(treeChildren[0], node);
                return;
            }

            List<double> attrs = new List<double>();
            foreach (TreeNode<double> c in treeChildren)
            {
                attrs.Add(c.Attribute);
            }
            VoronoiTreemapSingleLayer test;

            List<Site> Sites;
            int rep_count = 0;
            do{
                test = new VoronoiTreemapSingleLayer(attrs, treemapNode.Attribute, EThreshold, MaxIter);
                Sites = test.Compute();
                rep_count++;
            } while (test.Error > EThreshold && rep_count < 20);
            var err = test.Error;

            if (rep_count > 0)
            {
                if (err > _MaxError)
                    _MaxError = err;

                if (test.NumIter > _ActualIter)
                    _ActualIter = test.NumIter;
            }


            if (Sites.Count != count)
            {
                //something wrong
                return;
            }

            for (int i = 0; i < count; i++)
            {
                Site s = Sites[i];
                TreeNode<Polygon> node = new TreeNode<Polygon>(s.ClipPolyon);
                treemapNode.AddChild(node);
            }

            ParallelOptions options = new ParallelOptions();
            options.MaxDegreeOfParallelism = 3;
            System.Threading.Tasks.Parallel.For(0, count, options, i =>
            {
                ComputeProcess(treeChildren[i], treemapNode.Children[i]);
            });

        }

    }

}
