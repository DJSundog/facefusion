using GalaSoft.MvvmLight;
using GalaSoft.MvvmLight.Command;
using System.Windows.Threading;
using System;
using System.Linq;
using System.Collections.ObjectModel;
using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit;
using RelayCommand = GalaSoft.MvvmLight.Command.RelayCommand;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows;
using Microsoft.Kinect.Toolkit.Fusion;
using System.ComponentModel;
using System.Diagnostics;

namespace FaceFusion.ViewModels
{
    public class MainViewModel : ViewModelBase, IDisposable
    {
        #region Fields

        double _rotationRateInDegrees = 0;

        DispatcherTimer _elevationTimer;

        private WriteableBitmap _colorImageWritableBitmap;
        private byte[] _colorImageData;
        private byte[] _mappedColorImageData;

        private DepthImagePixel[] _depthImagePixels;
        private DepthImagePoint[] _colorMappedToDepthPoints;

        private byte[] _depthImageData;
        private WriteableBitmap _depthImageWritableBitmap;

        private DepthImagePixel[] _modDepthImagePixels;
        private byte[] _modDepthImageData;
        private WriteableBitmap _modDepthImageWritableBitmap;

        private ColorImageFormat _currentColorImageFormat = ColorImageFormat.Undefined;
        private DepthImageFormat _currentDepthImageFormat = DepthImageFormat.Undefined;

        Skeleton[] _skeletons;

        Skeleton _activeSkeleton;
        int _activeSkeletonId;

        public const int InactiveSkeletonId = -1;

        private int _colorWidth;
        private int _colorHeight;

        private int _depthWidth;
        private int _depthHeight;

        /// <summary>
        /// Track whether Dispose has been called
        /// </summary>
        private bool disposed;

        /// <summary>
        /// Bitmap that will hold color information
        /// </summary>
        private WriteableBitmap colorFusionBitmap;

        /// <summary>
        /// Intermediate storage for the depth data converted to color
        /// </summary>
        private int[] colorPixels;

        bool _isFusionInitialized;

        #region Fusion Fields

        /// <summary>
        /// The resolution of the depth image to be processed.
        /// </summary>
        private const DepthImageFormat DepthImageResolution = DepthImageFormat.Resolution320x240Fps30;

        /// <summary>
        /// The reconstruction volume voxel density in voxels per meter (vpm)
        /// 1000mm / 256vpm = ~3.9mm/voxel
        /// </summary>
        private const int VoxelsPerMeter = 512;

        /// <summary>
        /// The reconstruction volume voxel resolution in the X axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m wide
        /// </summary>
        private const int VoxelResolutionX = 128;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Y axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m high
        /// </summary>
        private const int VoxelResolutionY = 256;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Z axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m deep
        /// </summary>
        private const int VoxelResolutionZ = 128;


        private Matrix4 cameraTransform = Matrix4.Identity;

        /// <summary>
        /// The transformation between the world and camera view coordinate system
        /// </summary>
        private Matrix4 worldToCameraTransform;

        /// <summary>
        /// The reconstruction volume processor type. This parameter sets whether AMP or CPU processing
        /// is used. Note that CPU processing will likely be too slow for real-time processing.
        /// </summary>
        private const ReconstructionProcessor ProcessorType = ReconstructionProcessor.Amp;

        /// <summary>
        /// The zero-based device index to choose for reconstruction processing if the 
        /// ReconstructionProcessor AMP options are selected.
        /// Here we automatically choose a device to use for processing by passing -1, 
        /// </summary>
        private const int DeviceToUse = -1;

        /// <summary>
        /// The default transformation between the world and volume coordinate system
        /// </summary>
        private Matrix4 defaultWorldToVolumeTransform;

        /// <summary>
        /// The Kinect Fusion volume
        /// </summary>
        private Reconstruction volume;

        /// <summary>
        /// Parameter to translate the reconstruction based on the minimum depth setting. When set to
        /// false, the reconstruction volume +Z axis starts at the camera lens and extends into the scene.
        /// Setting this true in the constructor will move the volume forward along +Z away from the
        /// camera by the minimum depth threshold to enable capture of very small reconstruction volumes
        /// by setting a non-identity world-volume transformation in the ResetReconstruction call.
        /// Small volumes should be shifted, as the Kinect hardware has a minimum sensing limit of ~0.35m,
        /// inside which no valid depth is returned, hence it is difficult to initialize and track robustly  
        /// when the majority of a small volume is inside this distance.
        /// </summary>
        private bool translateResetPoseByMinDepthThreshold = true;

        /// <summary>
        /// Intermediate storage for the depth float data converted from depth image frame
        /// </summary>
        private FusionFloatImageFrame depthFloatBuffer;

        /// <summary>
        /// Intermediate storage for the point cloud data converted from depth float image frame
        /// </summary>
        private FusionPointCloudImageFrame pointCloudBuffer;

        /// <summary>
        /// Raycast shaded surface image
        /// </summary>
        private FusionColorImageFrame shadedSurfaceColorFrame;

        /// <summary>
        /// Get the image size of fusion images and bitmap.
        /// </summary>
        public static Size ImageSize
        {
            get
            {
                return GetImageSize(DepthImageResolution);
            }
        }

        /// <summary>
        /// Minimum depth distance threshold in meters. Depth pixels below this value will be
        /// returned as invalid (0). Min depth must be positive or 0.
        /// </summary>
        private float minDepthClip = FusionDepthProcessor.DefaultMinimumDepth;

        /// <summary>
        /// Maximum depth distance threshold in meters. Depth pixels above this value will be
        /// returned as invalid (0). Max depth must be greater than 0.
        /// </summary>
        private float maxDepthClip = FusionDepthProcessor.DefaultMaximumDepth;

        /// <summary>
        /// The timer to calculate FPS
        /// </summary>
        private DispatcherTimer fpsTimer;

        private int rawFrameCount;

        /// <summary>
        /// The count of the frames processed in the FPS interval
        /// </summary>
        private int processedFrameCount;

        /// <summary>
        /// The tracking error count
        /// </summary>
        private int trackingErrorCount;

        /// <summary>
        /// The seconds interval to calculate FPS
        /// </summary>
        private const int FpsInterval = 1;

        /// <summary>
        /// The sensor depth frame data length
        /// </summary>
        private int frameDataLength;

        /// <summary>
        /// The count of the depth frames to be processed
        /// </summary>
        private bool processingFrame;

        /// <summary>
        /// Max tracking error count, we will reset the reconstruction if tracking errors
        /// reach this number
        /// </summary>
        private const int MaxTrackingErrors = 100;

        /// <summary>
        /// If set true, will automatically reset the reconstruction when MaxTrackingErrors have occurred
        /// </summary>
        private const bool AutoResetReconstructionWhenLost = false;

        /// <summary>
        /// The integration weight.
        /// </summary>
        public const int IntegrationWeight = 30;

        #endregion

        #endregion

        #region Properties

        #region Commands

        public RelayCommand StartCommand { get; private set; }
        public RelayCommand PauseCommand { get; private set; }
        public RelayCommand ResetCommand { get; private set; }

        #endregion

        #region ZOffset

        /// <summary>
        /// The <see cref="ZOffset" /> property's name.
        /// </summary>
        public const string ZOffsetPropertyName = "ZOffset";

        private double _zOffset = 0.7;

        /// <summary>
        /// Gets the ZOffset property.
        /// </summary>
        public double ZOffset
        {
            get
            {
                return _zOffset;
            }

            set
            {
                if (_zOffset == value)
                {
                    return;
                }

                var oldValue = _zOffset;
                _zOffset = value;

                ResetReconstruction();
                StatusMessage = "Reset; ZOffset now " + _zOffset;
                // Update bindings, no broadcast
                RaisePropertyChanged(ZOffsetPropertyName);
            }
        }

        #endregion

        #region KinectSensor

        /// <summary>
        /// The <see cref="KinectSensor" /> property's name.
        /// </summary>
        public const string KinectSensorPropertyName = "KinectSensor";

        private KinectSensor _kinectSensor = null;

        /// <summary>
        /// Gets the KinectSensor property.
        /// </summary>
        public KinectSensor KinectSensor
        {
            get
            {
                return _kinectSensor;
            }

            set
            {
                if (_kinectSensor == value)
                {
                    return;
                }

                var oldValue = _kinectSensor;
                _kinectSensor = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(KinectSensorPropertyName);
            }
        }

        #endregion

        #region KinectSensorChooser

        /// <summary>
        /// The <see cref="KinectSensorChooser" /> property's name.
        /// </summary>
        public const string KinectSensorChooserPropertyName = "KinectSensorChooser";

        private KinectSensorChooser _kinectSensorChooser = new KinectSensorChooser();

        /// <summary>
        /// Gets the KinectSensorChooser property.
        /// </summary>
        public KinectSensorChooser KinectSensorChooser
        {
            get
            {
                return _kinectSensorChooser;
            }
        }

        #endregion

        #region ElevationAngle

        /// <summary>
        /// The <see cref="ElevationAngle" /> property's name.
        /// </summary>
        public const string ElevationAnglePropertyName = "ElevationAngle";

        private double _elevationAngle = 0.0;

        /// <summary>
        /// Gets the ElevationAngle property.
        /// </summary>
        public double ElevationAngle
        {
            get
            {
                return _elevationAngle;
            }

            set
            {
                if (_elevationAngle == value)
                {
                    return;
                }

                var oldValue = _elevationAngle;
                _elevationAngle = value;

                _elevationTimer.Stop();
                _elevationTimer.Start();

                // Update bindings, no broadcast
                RaisePropertyChanged(ElevationAnglePropertyName);
            }
        }

        #endregion

        #region ColorImage

        /// <summary>
        /// The <see cref="ColorImage" /> property's name.
        /// </summary>
        public const string ColorImagePropertyName = "ColorImage";

        private ImageSource _colorImage = null;

        /// <summary>
        /// Gets the ColorImage property.
        /// </summary>
        public ImageSource ColorImage
        {
            get
            {
                return _colorImage;
            }

            set
            {
                if (_colorImage == value)
                {
                    return;
                }

                var oldValue = _colorImage;
                _colorImage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(ColorImagePropertyName);
            }
        }

        #endregion

        #region DepthImage

        /// <summary>
        /// The <see cref="DepthImage" /> property's name.
        /// </summary>
        public const string DepthImagePropertyName = "DepthImage";

        private ImageSource _depthImage = null;

        /// <summary>
        /// Gets the DepthImage property.
        /// </summary>
        public ImageSource DepthImage
        {
            get
            {
                return _depthImage;
            }

            set
            {
                if (_depthImage == value)
                {
                    return;
                }

                var oldValue = _depthImage;
                _depthImage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(DepthImagePropertyName);
            }
        }

        #endregion

        #region FusionInputImage

        /// <summary>
        /// The <see cref="FusionInputImage" /> property's name.
        /// </summary>
        public const string FusionInputImagePropertyName = "FusionInputImage";

        private ImageSource _fusionInputImage = null;

        /// <summary>
        /// Gets the FusionInputImage property.
        /// </summary>
        public ImageSource FusionInputImage
        {
            get
            {
                return _fusionInputImage;
            }

            set
            {
                if (_fusionInputImage == value)
                {
                    return;
                }

                var oldValue = _fusionInputImage;
                _fusionInputImage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(FusionInputImagePropertyName);
            }
        }

        #endregion

        #region FusionOutputImage

        /// <summary>
        /// The <see cref="FusionOutputImage" /> property's name.
        /// </summary>
        public const string FusionOutputImagePropertyName = "FusionOutputImage";

        private ImageSource _fusionOutputImage = null;

        /// <summary>
        /// Gets the FusionOutputImage property.
        /// </summary>
        public ImageSource FusionOutputImage
        {
            get
            {
                return _fusionOutputImage;
            }

            set
            {
                if (_fusionOutputImage == value)
                {
                    return;
                }

                var oldValue = _fusionOutputImage;
                _fusionOutputImage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(FusionOutputImagePropertyName);
            }
        }

        #endregion

        #region StatusMessage

        /// <summary>
        /// The <see cref="StatusMessage" /> property's name.
        /// </summary>
        public const string StatusMessagePropertyName = "StatusMessage";

        private string _statusMessage = "";

        /// <summary>
        /// Gets the StatusMessage property.
        /// </summary>
        public string StatusMessage
        {
            get
            {
                return _statusMessage;
            }

            set
            {
                if (_statusMessage == value)
                {
                    return;
                }

                var oldValue = _statusMessage;
                _statusMessage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(StatusMessagePropertyName);
            }
        }

        #endregion

        #endregion

        #region Constructors

        public MainViewModel()
        {
            ////if (IsInDesignMode)
            ////{
            ////    // Code runs in Blend --> create design time data.
            ////}
            ////else
            ////{
            ////    // Code runs "for real": Connect to service, etc...
            ////}

            Init();
        }

        ~MainViewModel()
        {
            this.Dispose(false);
        }

        #endregion

        #region Overridden Methods

        public override void Cleanup()
        {
            // Clean own resources if needed
            base.Cleanup();
            Dispose();
        }

        /// <summary>
        /// Dispose the allocated frame buffers and reconstruction.
        /// </summary>
        public void Dispose()
        {
            this.Dispose(true);

            // This object will be cleaned up by the Dispose method.
            GC.SuppressFinalize(this);
        }

        /// <summary>
        /// Frees all memory associated with the FusionImageFrame.
        /// </summary>
        /// <param name="disposing">Whether the function was called from Dispose.</param>
        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                KinectSensorChooser.Stop();

                StopKinect();

                if (null != this.depthFloatBuffer)
                {
                    this.depthFloatBuffer.Dispose();
                }

                if (null != this.pointCloudBuffer)
                {
                    this.pointCloudBuffer.Dispose();
                }

                if (null != this.shadedSurfaceColorFrame)
                {
                    this.shadedSurfaceColorFrame.Dispose();
                }

                if (null != this.volume)
                {
                    this.volume.Dispose();
                }

                this.disposed = true;
            }
        }

        #endregion

        #region Private Methods

        private void Init()
        {
            _elevationTimer = new DispatcherTimer();
            _elevationTimer.Interval = TimeSpan.FromMilliseconds(500);
            _elevationTimer.Tick += new EventHandler(elevationTimer_Tick);

            InitRelayCommands();


            KinectSensorChooser.KinectChanged += SensorChooserOnKinectChanged;

            KinectSensorChooser.Start();
        }

        private void InitRelayCommands()
        {
            StartCommand = new RelayCommand(() =>
            {
                _rotationRateInDegrees = 3;
            });

            PauseCommand = new RelayCommand(() =>
            {
                _rotationRateInDegrees = 0;
                //var mesh = volume.CalculateMesh(1);
                //var verts = mesh.GetVertices();
                //var tris = mesh.GetTriangleIndexes();
                //StatusMessage = "verts: " + verts.Count + "  tris: " + tris.Count;
                //Debug.WriteLine("verts: " + verts.Count + "  tris: " + tris.Count);
            });

            ResetCommand = new RelayCommand(() =>
            {
                if (this.KinectSensor == null)
                {
                    StatusMessage = Properties.Resources.ConnectDeviceFirst;
                    return;
                }

                // reset the reconstruction and update the status text
                this.ResetReconstruction();
                StatusMessage = Properties.Resources.ResetReconstruction;
            });
        }

        private void SensorChooserOnKinectChanged(object sender, KinectChangedEventArgs kinectChangedEventArgs)
        {
            KinectSensor oldSensor = kinectChangedEventArgs.OldSensor;
            KinectSensor newSensor = kinectChangedEventArgs.NewSensor;

            if (oldSensor != null && oldSensor == KinectSensor)
            {
                StopKinect();
            }

            if (newSensor != null)
            {
                StartKinect(newSensor);
            }
        }

        private void StartKinect(KinectSensor newSensor)
        {
            try
            {
                newSensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                newSensor.DepthStream.Enable(DepthImageResolution);
                try
                {
                    // This will throw on non Kinect For Windows devices.
                    newSensor.DepthStream.Range = DepthRange.Near;
                    newSensor.SkeletonStream.EnableTrackingInNearRange = true;
                }
                catch (InvalidOperationException)
                {
                    newSensor.DepthStream.Range = DepthRange.Default;
                    newSensor.SkeletonStream.EnableTrackingInNearRange = false;
                }

                newSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                newSensor.SkeletonStream.Enable();
                newSensor.AllFramesReady += KinectSensorOnAllFramesReady;

                _elevationAngle = newSensor.ElevationAngle;
                RaisePropertyChanged(ElevationAnglePropertyName);

                this.KinectSensor = newSensor;

                InitFusion();
            }
            catch (InvalidOperationException)
            {
                // This exception can be thrown when we are trying to
                // enable streams on a device that has gone away.  This
                // can occur, say, in app shutdown scenarios when the sensor
                // goes away between the time it changed status and the
                // time we get the sensor changed notification.
                //
                // Behavior here is to just eat the exception and assume
                // another notification will come along if a sensor
                // comes back.
            }
        }

        private void StopKinect()
        {
            if (KinectSensor == null)
                return;

            KinectSensor.AllFramesReady -= KinectSensorOnAllFramesReady;
            KinectSensor.ColorStream.Disable();
            KinectSensor.DepthStream.Disable();
            KinectSensor.DepthStream.Range = DepthRange.Default;
            KinectSensor.SkeletonStream.Disable();
            KinectSensor.SkeletonStream.EnableTrackingInNearRange = false;
            KinectSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;

            KinectSensor.Stop();

            KinectSensor = null;
        }

        private void KinectSensorOnAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            bool colorFrameUpdated = false;
            bool depthFrameUpdated = false;
            bool skeletonFrameUpdated = false;

            using (var colorImageFrame = e.OpenColorImageFrame())
            {
                if (colorImageFrame != null)
                {

                    // Make a copy of the color frame for displaying.
                    var haveNewFormat = _currentColorImageFormat != colorImageFrame.Format;
                    if (haveNewFormat)
                    {
                        _colorWidth = colorImageFrame.Width;
                        _colorHeight = colorImageFrame.Height;
                        _currentColorImageFormat = colorImageFrame.Format;

                        _colorMappedToDepthPoints = new DepthImagePoint[_colorWidth * _colorHeight];

                        _colorImageData = new byte[colorImageFrame.PixelDataLength];

                    }

                    colorImageFrame.CopyPixelDataTo(this._colorImageData);
                    colorFrameUpdated = true;
                }
            }

            using (var depthImageFrame = e.OpenDepthImageFrame())
            {
                if (depthImageFrame != null)
                {

                    // Make a copy of the color frame for displaying.
                    var haveNewFormat = this._currentDepthImageFormat != depthImageFrame.Format;
                    if (haveNewFormat)
                    {
                        this._currentDepthImageFormat = depthImageFrame.Format;
                        _depthWidth = depthImageFrame.Width;
                        _depthHeight = depthImageFrame.Height;

                        this._depthImagePixels = new DepthImagePixel[depthImageFrame.PixelDataLength];
                        this._modDepthImagePixels = new DepthImagePixel[depthImageFrame.PixelDataLength];

                        this._depthImageData = new byte[depthImageFrame.PixelDataLength * 4];
                        this._modDepthImageData = new byte[depthImageFrame.PixelDataLength * 4];
                        this._mappedColorImageData = new byte[depthImageFrame.PixelDataLength * 4];

                        this._depthImageWritableBitmap = new WriteableBitmap(
                            _depthWidth, _depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        this._modDepthImageWritableBitmap = new WriteableBitmap(
                            _depthWidth, _depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        _colorImageWritableBitmap = new WriteableBitmap(
                            _depthWidth, _depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        ColorImage = _colorImageWritableBitmap;

                        DepthImage = this._depthImageWritableBitmap;
                        FusionInputImage = this._modDepthImageWritableBitmap;
                    }

                    depthImageFrame.CopyDepthImagePixelDataTo(this._depthImagePixels);

                    depthFrameUpdated = true;

                }
            }

            using (var skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    if (_skeletons == null || _skeletons.Length != skeletonFrame.SkeletonArrayLength)
                    {
                        _skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    }

                    skeletonFrame.CopySkeletonDataTo(_skeletons);
                    skeletonFrameUpdated = true;
                }
            }

            if (colorFrameUpdated && depthFrameUpdated && skeletonFrameUpdated)
            {
                rawFrameCount++;
                ProcessSkeletonFrame();
                ProcessColorFrame();
                ProcessDepthFrame();
            }
        }

        private void ProcessSkeletonFrame()
        {
            var skeletonList = _skeletons.ToList();
            var closestSkeleton = skeletonList.Where(s => s.TrackingState == SkeletonTrackingState.Tracked)
                                              .OrderBy(s => s.Position.Z * Math.Abs(s.Position.X))
                                              .FirstOrDefault();

            _activeSkeleton = closestSkeleton;
            if (closestSkeleton == null)
            {
                _activeSkeletonId = InactiveSkeletonId;
            }
            else
            {
                _activeSkeletonId = skeletonList.IndexOf(closestSkeleton) + 1;
            }
        }

        private unsafe void ProcessDepthFrame()
        {
            int width = _depthWidth;
            int height = _depthHeight;

            Array.Clear(_modDepthImagePixels, 0, _modDepthImagePixels.Length);
            short defaultDepth = (short)KinectSensor.DepthStream.UnknownDepth;
            var defaultDIP = new DepthImagePixel() { Depth = defaultDepth };

            double maxDepth = 4000;
            double minDepth = 400;

            fixed (byte* depthPtrFixed = _depthImageData, modDepthPtrFixed = _modDepthImageData)
            {
                int* depthIntPtr = (int*)depthPtrFixed;
                int* modDepthIntPtr = (int*)modDepthPtrFixed;

                int len = width * height;

                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        int srcIndex = (width - 1 - x) + y * width;
                        int targetIndex = x + y * width;

                        var dip = _depthImagePixels[srcIndex];
                        short depth = dip.Depth;
                        int playerIndex = dip.PlayerIndex;

                        byte value = (byte)(255 - 255 * (depth - minDepth) / maxDepth);

                        byte modValue = 0;

                        if (dip.IsKnownDepth && dip.PlayerIndex == _activeSkeletonId)
                        {
                            modValue = value;
                            _modDepthImagePixels[targetIndex] = new DepthImagePixel() { Depth = depth };
                        }
                        else
                        {
                            _modDepthImagePixels[targetIndex] = defaultDIP;
                        }

                        if (!dip.IsKnownDepth)
                        {
                            value = 0;
                        }

                        //_depthImageData[index * 4 + 0] = value;
                        //_depthImageData[index * 4 + 1] = value;
                        //_depthImageData[index * 4 + 2] = value;

                        int shiftValue = ((255 << 24) | (value << 16) | (value << 8) | (value));

                        *depthIntPtr = shiftValue;
                        depthIntPtr++;

                        //_modDepthImageData[index * 4 + 0] = modValue;
                        //_modDepthImageData[index * 4 + 1] = modValue;
                        //_modDepthImageData[index * 4 + 2] = modValue;

                        int shiftModValue = ((255 << 24) | (modValue << 16) | (modValue << 8) | (modValue));

                        *modDepthIntPtr = shiftModValue;
                        modDepthIntPtr++;

                    }
                }
            }

            _depthImageWritableBitmap.WritePixels(
                new Int32Rect(0, 0, width, height),
                _depthImageData,
                width * 4,
                0);

            _modDepthImageWritableBitmap.WritePixels(
                new Int32Rect(0, 0, width, height),
                _modDepthImageData,
                width * 4,
                0);

            ProcessFusionFrame((DepthImagePixel[])_depthImagePixels.Clone());
        }

        private unsafe void ProcessColorFrame()
        {
            int width = _colorWidth;
            int height = _colorHeight;
            int depthWidth = _depthWidth;
            int depthHeight = _depthHeight;

            var mapper = KinectSensor.CoordinateMapper;

            mapper.MapColorFrameToDepthFrame(_currentColorImageFormat, _currentDepthImageFormat, _depthImagePixels, _colorMappedToDepthPoints);

            Array.Clear(_mappedColorImageData, 0, _mappedColorImageData.Length);

            fixed (byte* colorPtrFixed = _colorImageData, mappedColorPtrFixed = _mappedColorImageData)
            {
                int* colorIntPtr = (int*)colorPtrFixed;
                int* mappedColorIntPtr = (int*)mappedColorPtrFixed;

                for (int y = 0; y < height; y++)
                {
                    for (int x = 0; x < width; x++)
                    {
                        int index = x + y * width;

                        var coord = _colorMappedToDepthPoints[index];
                        int cx = coord.X;
                        int cy = coord.Y;
                        if (cx >= 0 && cx < depthWidth &&
                            cy >= 0 && cy < depthHeight)
                        {
                            int targetIndex = (depthWidth - 1 - cx) + cy * depthWidth;

                            *(mappedColorIntPtr + targetIndex) = *(colorIntPtr + index);
                        }
                    }
                }
            }

            _colorImageWritableBitmap.WritePixels(
                new Int32Rect(0, 0, _depthWidth, _depthHeight),
                _mappedColorImageData,
                _depthWidth * 4,
                0);
        }

        void elevationTimer_Tick(object sender, EventArgs e)
        {
            _elevationTimer.Stop();

            if (KinectSensor != null)
            {
                KinectSensor.ElevationAngle = (int)Math.Round(ElevationAngle);
            }
        }

        /// <summary>
        /// Get the depth image size from the input depth image format.
        /// </summary>
        /// <param name="imageFormat">The depth image format.</param>
        /// <returns>The widht and height of the input depth image format.</returns>
        private static Size GetImageSize(DepthImageFormat imageFormat)
        {
            switch (imageFormat)
            {
                case DepthImageFormat.Resolution320x240Fps30:
                    return new Size(320, 240);

                case DepthImageFormat.Resolution640x480Fps30:
                    return new Size(640, 480);

                case DepthImageFormat.Resolution80x60Fps30:
                    return new Size(80, 60);
            }

            throw new ArgumentOutOfRangeException("imageFormat");
        }

        #region Fusion

        private void InitFusion()
        {
            if (_isFusionInitialized)
                return;

            _isFusionInitialized = true;

            this.frameDataLength = KinectSensor.DepthStream.FramePixelDataLength;

            // Allocate space to put the color pixels we'll create
            this.colorPixels = new int[this.frameDataLength];

            // This is the bitmap we'll display on-screen
            this.colorFusionBitmap = new WriteableBitmap(
                (int)ImageSize.Width,
                (int)ImageSize.Height,
                96.0,
                96.0,
                PixelFormats.Bgr32,
                null);

            FusionOutputImage = colorFusionBitmap;

            var volParam = new ReconstructionParameters(VoxelsPerMeter, VoxelResolutionX, VoxelResolutionY, VoxelResolutionZ);

            // Set the world-view transform to identity, so the world origin is the initial camera location.
            this.worldToCameraTransform = Matrix4.Identity;

            try
            {
                // This creates a volume cube with the Kinect at center of near plane, and volume directly
                // in front of Kinect.
                this.volume = Reconstruction.FusionCreateReconstruction(volParam, ProcessorType, DeviceToUse, this.worldToCameraTransform);

                this.defaultWorldToVolumeTransform = this.volume.GetCurrentWorldToVolumeTransform();

                if (this.translateResetPoseByMinDepthThreshold)
                {
                    this.ResetReconstruction();
                }
            }
            catch (ArgumentException ex)
            {
                StatusMessage = "ArgumentException - DX11 GPU not found?";
                return;
            }
            catch (InvalidOperationException ex)
            {
                StatusMessage = ex.Message;
                return;
            }
            catch (DllNotFoundException)
            {
                StatusMessage = Properties.Resources.MissingPrerequisite;
                return;
            }

            // Depth frames generated from the depth input
            this.depthFloatBuffer = new FusionFloatImageFrame((int)ImageSize.Width, (int)ImageSize.Height);

            // Point cloud frames generated from the depth float input
            this.pointCloudBuffer = new FusionPointCloudImageFrame((int)ImageSize.Width, (int)ImageSize.Height);

            // Create images to raycast the Reconstruction Volume
            this.shadedSurfaceColorFrame = new FusionColorImageFrame((int)ImageSize.Width, (int)ImageSize.Height);

            // Initialize and start the FPS timer
            this.fpsTimer = new DispatcherTimer();
            this.fpsTimer.Tick += new EventHandler(this.FpsTimerTick);
            this.fpsTimer.Interval = new TimeSpan(0, 0, FpsInterval);

            this.fpsTimer.Start();

            // Reset the reconstruction
            this.ResetReconstruction();

        }

        /// <summary>
        /// Reset the reconstruction to initial value
        /// </summary>
        private void ResetReconstruction()
        {
            // Reset tracking error counter
            this.trackingErrorCount = 0;

            // Set the world-view transform to identity, so the world origin is the initial camera location.
            this.worldToCameraTransform = Matrix4.Identity;
            this.cameraTransform = Matrix4.Identity;

            if (null != this.volume)
            {
                // Translate the reconstruction volume location away from the world origin by an amount equal
                // to the minimum depth threshold. This ensures that some depth signal falls inside the volume.
                // If set false, the default world origin is set to the center of the front face of the 
                // volume, which has the effect of locating the volume directly in front of the initial camera
                // position with the +Z axis into the volume along the initial camera direction of view.
                if (this.translateResetPoseByMinDepthThreshold)
                {
                    Matrix4 worldToVolumeTransform = this.defaultWorldToVolumeTransform;

                    // Translate the volume in the Z axis by the minDepthThreshold distance
                    float minDist = (this.minDepthClip < this.maxDepthClip) ? this.minDepthClip : this.maxDepthClip;
                    //worldToVolumeTransform.M43 -= minDist * VoxelsPerMeter;
                    worldToVolumeTransform.M43 -= (float)(ZOffset * VoxelsPerMeter);

                    this.volume.ResetReconstruction(this.worldToCameraTransform, worldToVolumeTransform);
                }
                else
                {
                    this.volume.ResetReconstruction(this.worldToCameraTransform);
                }
            }

            if (null != this.fpsTimer)
            {
                // Reset the processed frame count and reset the FPS timer
                this.fpsTimer.Stop();
                this.rawFrameCount = 0;
                this.processedFrameCount = 0;
                this.fpsTimer.Start();
            }
        }

        /// <summary>
        /// Update the FPS reading in the status text bar
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void FpsTimerTick(object sender, EventArgs e)
        {
            double fusionFPS = this.processedFrameCount / (double)FpsInterval;
            double rawFPS = this.rawFrameCount / (double)FpsInterval;

            // Update the FPS reading

            StatusMessage = String.Format("Kinect FPS: {0} Fusion FPS: {1}", rawFPS, fusionFPS);

            // Reset the frame count
            this.processedFrameCount = 0;
            this.rawFrameCount = 0;
        }

        /// <summary>
        /// Event handler for Kinect sensor's DepthFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ProcessFusionFrame(DepthImagePixel[] depthPixels)
        {
            if (!this.processingFrame)
            {
                // Mark that one frame will be processed
                this.processingFrame = true;

                BackgroundWorker worker = new BackgroundWorker();

                worker.DoWork += (s, ee) =>
                    {
                        ProcessFusionFrameBackground(depthPixels);
                    };

                worker.RunWorkerCompleted += (s, e) =>
                    {
                        // Write the pixel data into our bitmap
                        this.colorFusionBitmap.WritePixels(
                            new Int32Rect(0, 0, this.colorFusionBitmap.PixelWidth, this.colorFusionBitmap.PixelHeight),
                            this.colorPixels,
                            this.colorFusionBitmap.PixelWidth * sizeof(int),
                            0);

                        this.processingFrame = false;
                    };


                worker.RunWorkerAsync();

            }
        }

        /// <summary>
        /// Process the depth input
        /// </summary>
        /// <param name="depthPixels">The depth data array to be processed</param>
        private void ProcessFusionFrameBackground(DepthImagePixel[] depthPixels)
        {
            Debug.Assert(null != this.volume, "volume should be initialized");
            Debug.Assert(null != this.shadedSurfaceColorFrame, "shaded surface should be initialized");
            Debug.Assert(null != this.colorFusionBitmap, "color bitmap should be initialized");

            try
            {
                // Convert the depth image frame to depth float image frame
                FusionDepthProcessor.DepthToDepthFloatFrame(
                    depthPixels,
                    (int)ImageSize.Width,
                    (int)ImageSize.Height,
                    this.depthFloatBuffer,
                    FusionDepthProcessor.DefaultMinimumDepth,
                    FusionDepthProcessor.DefaultMaximumDepth,
                    false);

                float alignmentEnergy;
                this.worldToCameraTransform = volume.GetCurrentWorldToCameraTransform();

                bool trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                        depthFloatBuffer,
                        FusionDepthProcessor.DefaultAlignIterationCount,
                        null,
                        out alignmentEnergy,
                        this.worldToCameraTransform);

                this.worldToCameraTransform = volume.GetCurrentWorldToCameraTransform();

                this.volume.IntegrateFrame(depthFloatBuffer, IntegrationWeight, this.worldToCameraTransform);

                // ProcessFrame will first calculate the camera pose and then integrate
                // if tracking is successful
                //bool trackingSucceeded = this.volume.ProcessFrame(
                //    this.depthFloatBuffer,
                //    FusionDepthProcessor.DefaultAlignIterationCount,
                //    IntegrationWeight,
                //    this.volume.GetCurrentWorldToCameraTransform());

                // If camera tracking failed, no data integration or raycast for reference
                // point cloud will have taken place, and the internal camera pose
                // will be unchanged.
                if (!trackingSucceeded)
                {
                    this.trackingErrorCount++;

                    // Show tracking error on status bar
                    StatusMessage = Properties.Resources.CameraTrackingFailed;
                }
                else
                {
                    Matrix4 calculatedCameraPose = this.volume.GetCurrentWorldToCameraTransform();

                    // Set the camera pose and reset tracking errors
                    this.worldToCameraTransform = calculatedCameraPose;
                    this.trackingErrorCount = 0;
                }

                if (AutoResetReconstructionWhenLost && !trackingSucceeded && this.trackingErrorCount == MaxTrackingErrors)
                {
                    // Auto Reset due to bad tracking
                    StatusMessage = Properties.Resources.ResetVolume;

                    // Automatically Clear Volume and reset tracking if tracking fails
                    this.ResetReconstruction();
                }
                var c = this.cameraTransform;
                System.Windows.Media.Media3D.Matrix3D m = new System.Windows.Media.Media3D.Matrix3D(c.M11, c.M12, c.M13, c.M14,
                                                                                                    c.M21, c.M22, c.M23, c.M24,
                                                                                                    c.M31, c.M32, c.M33, c.M34,
                                                                                                    c.M41, c.M42, c.M43, c.M44);

                double zTranslate = 0.5 * VoxelResolutionZ / VoxelsPerMeter + ZOffset;
                m.Translate(new System.Windows.Media.Media3D.Vector3D(0, 0, -zTranslate));
                m.Rotate(new System.Windows.Media.Media3D.Quaternion(new System.Windows.Media.Media3D.Vector3D(0, 1, 0), _rotationRateInDegrees));
                m.Translate(new System.Windows.Media.Media3D.Vector3D(0, 0, zTranslate));

                c.M11 = (float)m.M11;
                c.M12 = (float)m.M12;
                c.M13 = (float)m.M13;
                c.M14 = (float)m.M14;
                c.M21 = (float)m.M21;
                c.M22 = (float)m.M22;
                c.M23 = (float)m.M23;
                c.M24 = (float)m.M24;
                c.M31 = (float)m.M31;
                c.M32 = (float)m.M32;
                c.M33 = (float)m.M33;
                c.M34 = (float)m.M34;
                c.M41 = (float)m.OffsetX;
                c.M42 = (float)m.OffsetY;
                c.M43 = (float)m.OffsetZ;
                c.M44 = (float)m.M44;
                cameraTransform = c;

                // Calculate the point cloud
                this.volume.CalculatePointCloud(this.pointCloudBuffer, this.cameraTransform);

                // Shade point cloud and render
                FusionDepthProcessor.ShadePointCloud(
                    this.pointCloudBuffer,
                    this.cameraTransform,
                    this.shadedSurfaceColorFrame,
                    null);

                this.shadedSurfaceColorFrame.CopyPixelDataTo(this.colorPixels);

                // The input frame was processed successfully, increase the processed frame count
                ++this.processedFrameCount;
            }
            catch (InvalidOperationException ex)
            {
                StatusMessage = ex.Message;
            }
            finally
            {
            }
        }

        #endregion

        #endregion
    }
}