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
    public partial class FaceTrackingView : UserControl
    {
        public FaceTrackingViewModel FaceTrackingVM
        {
            get
            {
                return this.DataContext as FaceTrackingViewModel;
            }
        }

        public FaceTrackingView()
        {
            InitializeComponent();

            if (FaceTrackingVM != null)
            {
                FaceTrackingVM.FrameUpdated += FaceTrackingVM_FrameUpdated;
            }

            this.DataContextChanged += new DependencyPropertyChangedEventHandler(FaceTrackingView_DataContextChanged);
        }

        #region Overridden Methods

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);

            if (this.FaceTrackingVM != null)
            {
                foreach (FaceTrackerBase tracker in this.FaceTrackingVM.FaceTrackers)
                {
                    tracker.DrawFaceModel(drawingContext);
                }
            }
        }

        #endregion

        #region Private Methods

        void FaceTrackingVM_FrameUpdated(object sender, EventArgs e)
        {
            this.InvalidateVisual();
        }

        void FaceTrackingView_DataContextChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            var oldVM = e.OldValue as FaceTrackingViewModel;
            if (oldVM != null)
            {
                oldVM.FrameUpdated -= FaceTrackingVM_FrameUpdated;
            }

            var newVM = e.NewValue as FaceTrackingViewModel;
            if (newVM != null)
            {
                newVM.FrameUpdated += FaceTrackingVM_FrameUpdated;
            }
        }
        
        #endregion
        
    }
}
