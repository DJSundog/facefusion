// --------------------------------------------------------------------------------------------------------------------
// <copyright file="FaceTrackingViewer.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// --------------------------------------------------------------------------------------------------------------------

namespace FaceFusion.Views
{
    using System;
    using System.Linq;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit.FaceTracking;

    using Point = System.Windows.Point;
    using System.ComponentModel;

    /// <summary>
    /// Class that uses the Face Tracking SDK to display a face mask for
    /// tracked skeletons
    /// </summary>
    public partial class FaceTrackingViewer : UserControl, IDisposable
    {
        public static readonly DependencyProperty KinectProperty = DependencyProperty.Register(
            "Kinect",
            typeof(KinectSensor),
            typeof(FaceTrackingViewer),
            new PropertyMetadata(
                null, (o, args) => ((FaceTrackingViewer)o).OnSensorChanged((KinectSensor)args.OldValue, (KinectSensor)args.NewValue)));

        private const uint MaxMissedFrames = 100;

        private readonly Dictionary<int, SkeletonFaceTracker> skeletonFaceTrackers = new Dictionary<int, SkeletonFaceTracker>();

        private readonly List<RegionFaceTracker> regionFaceTrackers = new List<RegionFaceTracker>();

        private byte[] colorImage;

        private ColorImageFormat colorImageFormat = ColorImageFormat.Undefined;

        private DepthImagePixel[] depthImage;

        private DepthImageFormat depthImageFormat = DepthImageFormat.Undefined;

        private bool disposed;

        private Skeleton[] skeletonData;

        private FaceTracker faceTracker;

        public FaceTrackingViewer()
        {
            this.InitializeComponent();
        }

        ~FaceTrackingViewer()
        {
            this.Dispose(false);
        }

        public KinectSensor Kinect
        {
            get
            {
                return (KinectSensor)this.GetValue(KinectProperty);
            }

            set
            {
                this.SetValue(KinectProperty, value);
            }
        }

        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                this.ResetFaceTracking();

                this.disposed = true;
            }
        }

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            foreach (FaceTrackerBase tracker in this.skeletonFaceTrackers.Values)
            {
                tracker.DrawFaceModel(drawingContext);
            }
            foreach (FaceTrackerBase tracker in this.regionFaceTrackers)
            {
                tracker.DrawFaceModel(drawingContext);
            }
        }

        private void OnAllFramesReady(object sender, AllFramesReadyEventArgs allFramesReadyEventArgs)
        {
            ColorImageFrame colorImageFrame = null;
            DepthImageFrame depthImageFrame = null;
            SkeletonFrame skeletonFrame = null;

            try
            {
                colorImageFrame = allFramesReadyEventArgs.OpenColorImageFrame();
                depthImageFrame = allFramesReadyEventArgs.OpenDepthImageFrame();
                skeletonFrame = allFramesReadyEventArgs.OpenSkeletonFrame();

                if (colorImageFrame == null || depthImageFrame == null || skeletonFrame == null)
                {
                    return;
                }

                // Check for image format changes.  The FaceTracker doesn't
                // deal with that so we need to reset.
                if (this.depthImageFormat != depthImageFrame.Format)
                {
                    this.ResetFaceTracking();
                    this.depthImage = null;
                    this.depthImageFormat = depthImageFrame.Format;
                }

                if (this.colorImageFormat != colorImageFrame.Format)
                {
                    this.ResetFaceTracking();
                    this.colorImage = null;
                    this.colorImageFormat = colorImageFrame.Format;
                }

                // Create any buffers to store copies of the data we work with
                if (this.depthImage == null)
                {
                    this.depthImage = new DepthImagePixel[depthImageFrame.PixelDataLength];
                }

                if (this.colorImage == null)
                {
                    this.colorImage = new byte[colorImageFrame.PixelDataLength];
                }

                // Get the skeleton information
                if (this.skeletonData == null || this.skeletonData.Length != skeletonFrame.SkeletonArrayLength)
                {
                    this.skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                }

                colorImageFrame.CopyPixelDataTo(this.colorImage);
                depthImageFrame.CopyDepthImagePixelDataTo(this.depthImage);
                skeletonFrame.CopySkeletonDataTo(this.skeletonData);

                // Update the list of trackers and the trackers with the current frame information
                foreach (Skeleton skeleton in this.skeletonData)
                {
                    if (skeleton.TrackingState == SkeletonTrackingState.Tracked
                        || skeleton.TrackingState == SkeletonTrackingState.PositionOnly)
                    {
                        // We want keep a record of any skeleton, tracked or untracked.
                        if (!this.skeletonFaceTrackers.ContainsKey(skeleton.TrackingId))
                        {
                            this.skeletonFaceTrackers.Add(skeleton.TrackingId, new SkeletonFaceTracker());
                        }

                        // Give each tracker the upated frame.
                        SkeletonFaceTracker skeletonFaceTracker;
                        if (this.skeletonFaceTrackers.TryGetValue(skeleton.TrackingId, out skeletonFaceTracker))
                        {
                            skeletonFaceTracker.OnFrameReady(this.Kinect, colorImageFormat, colorImage, depthImageFormat, depthImage, skeleton);
                            skeletonFaceTracker.LastTrackedFrame = depthImageFrame.FrameNumber;
                        }
                    }
                }

                foreach (var tracker in regionFaceTrackers)
                {
                    if (IsRectBeingTracked(tracker.FaceRect))
                    {
                        //We now have a skeleton in this area, so remove the region face tracker
                        RemoveFaceTracker(tracker);
                    }
                    else
                    {
                        tracker.OnFrameReady(this.Kinect, colorImageFormat, colorImage, depthImageFormat, depthImage);
                        tracker.LastTrackedFrame = depthImageFrame.FrameNumber;
                    }
                }

                this.RemoveOldTrackers(depthImageFrame.FrameNumber);

                if (depthImageFrame.FrameNumber % 30 == 0)
                {
                    DetectFaces(colorImageFrame);
                }

                this.InvalidateVisual();
            }
            finally
            {
                if (colorImageFrame != null)
                {
                    colorImageFrame.Dispose();
                }

                if (depthImageFrame != null)
                {
                    depthImageFrame.Dispose();
                }

                if (skeletonFrame != null)
                {
                    skeletonFrame.Dispose();
                }
            }
        }

        private void DetectFaces(ColorImageFrame colorImageFrame)
        {
            if (this.faceTracker == null)
            {
                try
                {
                    this.faceTracker = new FaceTracker(this.Kinect);
                }
                catch (InvalidOperationException)
                {
                    // During some shutdown scenarios the FaceTracker
                    // is unable to be instantiated.  Catch that exception
                    // and don't track a face.
                    Debug.WriteLine("AllFramesReady - creating a new FaceTracker threw an InvalidOperationException");
                    this.faceTracker = null;
                }
            }
            Microsoft.Kinect.Toolkit.FaceTracking.Rect roi = new Microsoft.Kinect.Toolkit.FaceTracking.Rect(0, 0, colorImageFrame.Width, colorImageFrame.Height);


            BackgroundWorker worker = new BackgroundWorker();
            worker.DoWork += (s, e) =>
                {

                    var shortImage = Helpers.ConvertDepthImagePixelToShort(depthImage);

                    var rects = faceTracker.DetectFaces(colorImageFormat, colorImage, depthImageFormat, shortImage, roi);

                    e.Result = rects;
                };

            worker.RunWorkerCompleted += (s, e) =>
                {
                    var rects = e.Result as IEnumerable<WeightedRect>;
                    int count = rects.Count();
                    if (count > 0)
                    {
                        Trace.WriteLine("Face rects: " + count);
                        foreach (var rect in rects)
                        {
                            if (!IsRectBeingTracked(rect.Rect))
                            {
                                var tracker = new RegionFaceTracker(rect.Rect);
                                Trace.WriteLine("Starting to track face at " + rect.Rect.Left + ", " + rect.Rect.Top + " with weight " + rect.Weight);

                                this.regionFaceTrackers.Add(tracker);
                            }
                        }
                    }
                };
        }

        private bool IsRectBeingTracked(Microsoft.Kinect.Toolkit.FaceTracking.Rect rect)
        {
            foreach (var kvp in this.skeletonFaceTrackers)
            {
                var trackedRect = kvp.Value.FaceRect;
                var intersection = rect.Intersection(trackedRect);

                if (intersection.Width > 0 && intersection.Height > 0)
                    return true;
            }
            return false;
        }

        private void OnSensorChanged(KinectSensor oldSensor, KinectSensor newSensor)
        {
            if (oldSensor != null)
            {
                oldSensor.AllFramesReady -= this.OnAllFramesReady;
                this.ResetFaceTracking();
            }

            if (newSensor != null)
            {
                newSensor.AllFramesReady += this.OnAllFramesReady;
            }
        }

        /// <summary>
        /// Clear out any trackers for skeletons we haven't heard from for a while
        /// </summary>
        private void RemoveOldTrackers(int currentFrameNumber)
        {
            var trackersToRemove = new List<int>();

            foreach (var tracker in this.skeletonFaceTrackers)
            {
                uint missedFrames = (uint)currentFrameNumber - (uint)tracker.Value.LastTrackedFrame;
                if (missedFrames > MaxMissedFrames)
                {
                    // There have been too many frames since we last saw this skeleton
                    trackersToRemove.Add(tracker.Key);
                }
            }

            foreach (int trackingId in trackersToRemove)
            {
                this.RemoveTracker(trackingId);
            }


            var faceTrackersToRemove = new List<RegionFaceTracker>();

            foreach (var tracker in this.regionFaceTrackers)
            {
                uint missedFrames = (uint)currentFrameNumber - (uint)tracker.LastTrackedFrame;
                if (missedFrames > MaxMissedFrames)
                {
                    // There have been too many frames since we last saw this skeleton
                    faceTrackersToRemove.Add(tracker);
                }
            }

            foreach (var tracker in faceTrackersToRemove)
            {
                this.RemoveFaceTracker(tracker);
            }
        }

        private void RemoveTracker(int trackingId)
        {
            this.skeletonFaceTrackers[trackingId].Dispose();
            this.skeletonFaceTrackers.Remove(trackingId);
        }

        private void RemoveFaceTracker(RegionFaceTracker tracker)
        {
            tracker.Dispose();
            this.regionFaceTrackers.Remove(tracker);
        }

        private void ResetFaceTracking()
        {
            foreach (int trackingId in new List<int>(this.skeletonFaceTrackers.Keys))
            {
                this.RemoveTracker(trackingId);
            }
            foreach (var tracker in this.regionFaceTrackers.ToList())
            {
                this.RemoveFaceTracker(tracker);
            }

            if (this.faceTracker != null)
            {
                this.faceTracker.Dispose();
                this.faceTracker = null;
            }
        }

    }
}