using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Threading.Tasks;
using System.IO;
using System.Xml;
using System.Threading;
using System.Diagnostics;

using Lava.Data;
using Lava.Util;
using Lava.Visual;
using Lava.Visual.Layout;
using Lava.IO;

using Treemap.Voronoi.DataStructures;
using Treemap.Voronoi.Algorithm;

using System.Runtime.InteropServices;

namespace Treemap.Voronoi.Demo
{
    public class Test
    {
        private Display Display;
        private Visualization Vis;

        private ITree AttrTree;
        private INode Root;
        private List<double> Boundary;

        private ITree Treemap;
        private int MaxDepth;

        private Graph TreeGraph;

        public Test()
        {
            this.MaxDepth = 0;
            GetData();
        }

        public Test(Display _display) : this()
        {
            this.Display = _display;
            this.Vis = new Visualization(_display);
        }
        private void GetData()
        {
            //GetTreeFromXml("../../Demo/flare.xml");
            GetTreeFromCsv("../../Demo/OctagonLinkedList.csv");
            SetBoundary();
        }
        private void SetBoundary()
        {
            List<double> Boundary = new List<double>();

            Boundary.Add(263.6);
            Boundary.Add(0);

            Boundary.Add(649.26);
            Boundary.Add(0);

            Boundary.Add(900);
            Boundary.Add(263.6);

            Boundary.Add(900);
            Boundary.Add(649.26);

            Boundary.Add(649.26);
            Boundary.Add(900);

            Boundary.Add(263.6);
            Boundary.Add(900);

            Boundary.Add(0);
            Boundary.Add(649.26);

            Boundary.Add(0);
            Boundary.Add(263.6);

            for (int i = 0; i < Boundary.Count; i++)
            {
                Boundary[i] += 20;
            }
            this.Boundary = Boundary;
        }

        /// <summary>
        /// core of the test
        /// </summary>
        public void CalcView()
        {
            Tree<double> AttrTreeTmp = ConvertToTree<double>(AttrTree);
            TimeSpan averagttime = new TimeSpan();
            for (int i = 0; i < 10; i++)
            {
                Stopwatch sw = new Stopwatch();
                sw.Start();
                ///////////////////////////////////////////////////
                VoronoiTreemap test = new VoronoiTreemap(AttrTreeTmp, Boundary, 0.2, 300);
                Tree<Polygon> tmp = test.Compute();
                ///////////////////////////////////////////////////
                sw.Stop();
                averagttime += sw.Elapsed;
                var error = test.MaxError;
                var iters = test.ActualIter;
                Console.WriteLine("average calc time: {0} ", TimeSpan.FromMilliseconds(averagttime.TotalMilliseconds / (i + 1)));
                this.Treemap = ConvertToITree<Polygon>(tmp);
            }

            //ViewTreemap();
        }

        private double ComputeAttribute(INode tree_root)
        {
            int childcount = AttrTree.GetChildCount(tree_root);
            double attribute = tree_root.Get<double>("attribute");
            if (childcount == 0)
            {
                int depth = AttrTree.GetDepth(tree_root);
                if (depth > MaxDepth)
                    MaxDepth = depth;

                if (attribute > 0)
                    return attribute;
                else
                {
                    tree_root.Set<double>("attribute", 1);
                    return 1;
                }
            }
            var children = AttrTree.GetChildren(tree_root);
            attribute = 0;
            foreach (INode i in children)
            {
                attribute += ComputeAttribute(i);
            }
            //attribute += 1;
            tree_root.Set<double>("attribute", attribute);
            return attribute;
        }



        private void ViewTreemap()
        {
            var polygonTable = Vis.BuildVisualTable("poly", Display, 1);
            polygonTable.AddColumn<Brush>(Styles.StrokeBrush, BrushLib.FromColor(Colors.Black));
            polygonTable.AddColumn<Brush>(Styles.FillBrush, BrushLib.FromColor(Colors.White));
            polygonTable.EnsureStyles(Styles.X, Styles.Y, Styles.Opacity, Styles.StrokeWidth);
            polygonTable.AddColumn<List<double>>("polygon", null);
            polygonTable.AddColumn<double>("depth", 0);
            //polygonTable.AddColumn<INode>("node", null);

            int depth_idx = polygonTable.GetColumnIndex("depth");
            int polygon_idx = polygonTable.GetColumnIndex("polygon");
            //int node_idx = polygonTable.GetColumnIndex("node");

            var node_list = Treemap.BFS(Treemap.Root).Reverse<INode>();
            foreach (INode i in node_list)
            {
                int depth = Treemap.GetDepth(i);
                if (depth > 0)
                {

                    int height = MaxDepth - depth + 1;

                    int row = polygonTable.AddRow();
                    polygonTable.Set<List<double>>(row, polygon_idx, i.Get<Polygon>("attribute").ToListDouble());
                    //polygonTable.Set<INode>(row, node_idx, i);
                    polygonTable.Set<double>(row, Styles.StrokeWidth, 1.5 * height / MaxDepth);
                    if (depth > 2)
                        polygonTable.Set<Brush>(row, Styles.StrokeBrush, BrushLib.FromColor(ColorLib.NewAlpha(Colors.Black, 0.4)));
                    polygonTable.Set<Brush>(row, Styles.FillBrush, BrushLib.FromColor(Color.FromArgb(Convert.ToByte(40 + 20 * height / MaxDepth), Convert.ToByte(30 * depth / MaxDepth), 0, Convert.ToByte(255 * height / MaxDepth))));
                    //polygonTable.Set<Brush>(row, Styles.FillBrush, BrushLib.FromColor(ColorLib.NewAlpha(ColorLib.Category10[0], 0.3)));
                }
            }

            //polygonTable.AutoRepaint = true;
            var polygonRender = new GeometryRender(vi =>
            {
                var polygon = GeometryLib.Polygon(vi.Get<List<double>>(polygon_idx), true, true);
                return polygon;
            });

            polygonTable.SetupRender(polygonRender);
        }

        private void GetTreeFromXml(string s)
        {
            XmlDocument xmldoc = new XmlDocument();
            xmldoc.Load(s);
            var xml_root = xmldoc.DocumentElement;

            TreeGraph = new Graph(true);
            TreeGraph.NodeTable.AddColumn<double>("attribute", 0);
            var Root = TreeGraph.AddNode();
            GetChildrenFromXml(xml_root, Root);

            AttrTree = TreeGraph.GetSpanningTree(TreeGraph.GetNode(0));
            ComputeAttribute(AttrTree.Root);
        }

        private void GetChildrenFromXml(XmlElement xml_root, INode tree_root)
        {
            if (xml_root.Name == "node")
            {
                tree_root.Set<double>("attribute", Convert.ToDouble(xml_root.GetAttribute("attribute")));
                return;
            }
            var children = xml_root.ChildNodes;
            foreach (XmlElement x in children)
            {
                var child = TreeGraph.AddNode();
                TreeGraph.AddEdge(tree_root, child);
                GetChildrenFromXml(x, child);
            }
        }
       
        private void GetTreeFromCsv(string s)
        {
            var config = new TableReaderConfig();
            config.WithHeader = true;
            config.DelimitedString = ",";

            ITable csv = TableIO.FromComputer(s, config);

            Dictionary<int, int> dic = new Dictionary<int, int>();
            int n = csv.RowCount;

            TreeGraph = new Graph(true);
            TreeGraph.NodeTable.AddColumn<double>("attribute", 0);
            int count = 0;
            for (int i = 0; i < n; i++)
            {
                //
                //var a = csv.Get<double>(i, "polygon");
                if (0 != csv.Get<double>(i, "polygon"))
                {
                    var id = csv.Get<int>(i, "id");
                    dic.Add(id, count);
                    count++;
                    TreeGraph.AddNode();
                }
                    
            }

            n = TreeGraph.ItemCount;
            for (int i = 0; i < csv.RowCount; i++)
            {
                var id = csv.Get<int>(i, "id");
                var parentid = csv.Get<int>(i, "parentId");
                if (parentid == -1)
                    Root = TreeGraph.GetNode(dic[id]);
                else if (0 != csv.Get<double>(i, "polygon"))
                {
                    id = dic[id];
                    parentid = dic[parentid];
                    TreeGraph.AddEdge(parentid, id);
                }
                    
            }

            AttrTree = TreeGraph.GetSpanningTree(Root);
            ComputeAttribute(AttrTree.Root);
        }

        public Tree<T> ConvertToTree<T>(ITree itree)
        {
            INode root = itree.Root;
            Tree<T> tree = new Tree<T>(root.Get<T>("attribute"));
            ConvertToTreeProcess<T>(itree, root, tree.Root);
            return tree;
        }
        private void ConvertToTreeProcess<T>(ITree itree, INode node, TreeNode<T> treenode)
        {
            if (itree.GetChildCount(node) == 0)
                return;
            foreach (INode children in itree.GetChildren(node))
            {
                TreeNode<T> new_node = new TreeNode<T>(children.Get<T>("attribute"));
                treenode.AddChild(new_node);
                ConvertToTreeProcess<T>(itree, children, new_node);
            }
        }

        public ITree ConvertToITree<T>(Tree<T> tree)
        {
            Graph g = new Graph(true);
            g.NodeTable.AddColumn<T>("attribute");
            INode root = g.AddNode();
            root.Set<T>("attribute", tree.Root.Attribute);
            ConvertToITreeProcess<T>(tree.Root, g, root);
            return g.GetSpanningTree(root);
        }

        private void ConvertToITreeProcess<T>(TreeNode<T> treenode, Graph g, INode node)
        {
            if (treenode.Children.Count == 0)
                return;
            foreach (TreeNode<T> children in treenode.Children)
            {
                if (children != null)
                {
                    INode new_node = g.AddNode();
                    new_node.Set<T>("attribute", children.Attribute);
                    g.AddEdge(node, new_node);
                    ConvertToITreeProcess<T>(children, g, new_node);
                }
               
            }

        }

    }
}