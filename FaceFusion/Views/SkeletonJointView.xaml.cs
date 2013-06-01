using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using FaceFusion.ViewModels;

namespace FaceFusion.Views
{
    /// <summary>
    /// Interaction logic for SkeletonJointView.xaml
    /// </summary>
    public partial class SkeletonJointView : UserControl
    {
        public SkeletonJointViewModel ViewModel
        {
            get
            {
                return this.DataContext as SkeletonJointViewModel;
            }
        }

        public SkeletonJointView()
        {
            InitializeComponent();

            if (ViewModel != null)
            {
                ViewModel.FrameUpdated += FrameUpdated;
            }

            this.DataContextChanged += new DependencyPropertyChangedEventHandler(VM_DataContextChanged);
        }

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);

            if (ViewModel == null)
                return;


            var pen = new Pen(Brushes.Blue, 2.0);

            foreach (var joint in ViewModel.Joints)
            {
                drawingContext.DrawEllipse(null, pen, new Point(joint.X, joint.Y), 4, 4);
            }
        }

        void FrameUpdated(object sender, EventArgs e)
        {
            this.InvalidateVisual();
        }

        void VM_DataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            var oldVM = e.OldValue as SkeletonJointViewModel;
            if (oldVM != null)
            {
                oldVM.FrameUpdated -= FrameUpdated;
            }

            var newVM = e.NewValue as SkeletonJointViewModel;
            if (newVM != null)
            {
                newVM.FrameUpdated += FrameUpdated;
            }
        }
        
    }
}
