﻿using GalaSoft.MvvmLight;
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
using System.Collections.Generic;
using Blake.NUI.WPF.Utility;
using System.Windows.Media.Media3D;
using System.Threading;
using System.Runtime.InteropServices;

namespace FaceFusion.ViewModels
{
    public class MainViewModel : ViewModelBase, IDisposable
    {
        #region Fields

        #region Standard Fields

        DateTime _lastFPSUpdate = DateTime.Now;
        double _rotationRateInDegrees = 0;
        double _currentRotationDegrees = 0;

        DispatcherTimer _elevationTimer;

        private WriteableBitmap _colorImageWritableBitmap;
        private byte[] _mappedColorImageData;

        private WriteableBitmap _residualWritableBitmap;
        private byte[] _residualImageData;

        private SynchronizationContext _syncContext = SynchronizationContext.Current;

        private Pool<FusionWorkItem, DepthImageFormat> _fusionWorkItemPool;
        private WorkQueue<FusionWorkItem> _fusionWorkQueue;

        private Pool<KinectFrameWorkItem, KinectFormat> _kinectFrameWorkItemPool;
        private WorkQueue<KinectFrameWorkItem> _kinectWorkQueue;

        private byte[] _depthImageData;
        private WriteableBitmap _depthImageWritableBitmap;

        private byte[] _modDepthImageData;
        private WriteableBitmap _modDepthImageWritableBitmap;

        private const ColorImageFormat DefaultColorImageFormat = ColorImageFormat.RgbResolution640x480Fps30;
        private const DepthImageFormat DefaultDepthImageFormat = DepthImageFormat.Resolution320x240Fps30;
        private KinectFormat _currentKinectFormat;

        private const int DefaultNumSkeletons = 6;

        Skeleton _activeSkeleton;
        int _activeSkeletonId;

        public const int InactiveSkeletonId = -1;

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

        float _alignmentEnergy;

        int _activeSkeletonLostCount = 0;
        int _activeSkeletonLostLimit = 60;

        #endregion

        #region Fusion Fields

        Vector3 _volumeCenter = new Vector3();
        Vector3 _currentVolumeCenter = new Vector3();

        /// <summary>
        /// The reconstruction volume voxel density in voxels per meter (vpm)
        /// 1000mm / 256vpm = ~3.9mm/voxel
        /// </summary>
        private const int VoxelsPerMeter = 384;

        /// <summary>
        /// The reconstruction volume voxel resolution in the X axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m wide
        /// </summary>
        private const int VoxelResolutionX = 128;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Y axis
        /// At a setting of 256vpm the volume is 384 / 256 = 1.5m high
        /// </summary>
        private const int VoxelResolutionY = 128;

        /// <summary>
        /// The reconstruction volume voxel resolution in the Z axis
        /// At a setting of 256vpm the volume is 512 / 256 = 2m deep
        /// </summary>
        private const int VoxelResolutionZ = 128;

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
        /// The zero-based device targetIndex to choose for reconstruction processing if the 
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

        float[] _residualData;
        private FusionFloatImageFrame residualFloatBuffer;

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
        public Size DepthImageSize
        {
            get
            {
                return FormatHelper.GetDepthSize(DefaultDepthImageFormat);
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
        /// The sensor depth frame data length
        /// </summary>
        private int frameDataLength;

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
        public const int IntegrationWeight = 45;

        #endregion

        #endregion

        #region Properties

        #region Commands

        public RelayCommand StartCommand { get; private set; }
        public RelayCommand PauseCommand { get; private set; }
        public RelayCommand ResetCommand { get; private set; }

        #endregion

        #region SkeletonJointVM

        /// <summary>
        /// The <see cref="SkeletonJointVM" /> property's name.
        /// </summary>
        public const string SkeletonJointVMPropertyName = "SkeletonJointVM";

        private SkeletonJointViewModel _skeletonJointVM = new SkeletonJointViewModel();

        /// <summary>
        /// Gets the SkeletonJointVM property.
        /// </summary>
        public SkeletonJointViewModel SkeletonJointVM
        {
            get
            {
                return _skeletonJointVM;
            }

            set
            {
                if (_skeletonJointVM == value)
                {
                    return;
                }

                var oldValue = _skeletonJointVM;
                _skeletonJointVM = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(SkeletonJointVMPropertyName);
            }
        }

        #endregion

        #region UserFusionOnly

        /// <summary>
        /// The <see cref="UserFusionOnly" /> property's name.
        /// </summary>
        public const string UserFusionOnlyPropertyName = "UserFusionOnly";

        private bool _userFusionOnly = true;

        /// <summary>
        /// Gets the UserFusionOnly property.
        /// </summary>
        public bool UserFusionOnly
        {
            get
            {
                return _userFusionOnly;
            }

            set
            {
                if (_userFusionOnly == value)
                {
                    return;
                }

                var oldValue = _userFusionOnly;
                _userFusionOnly = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(UserFusionOnlyPropertyName);
                ResetReconstruction();
            }
        }

        #endregion

        #region AlignmentEnergyString

        /// <summary>
        /// The <see cref="AlignmentEnergyString" /> property's name.
        /// </summary>
        public const string AlignmentEnergyStringPropertyName = "AlignmentEnergyString";

        private string _alignmentEnergyString = "Alignment Energy: ";

        /// <summary>
        /// Gets the AlignmentEnergyString property.
        /// </summary>
        public string AlignmentEnergyString
        {
            get
            {
                return _alignmentEnergyString;
            }

            set
            {
                if (_alignmentEnergyString == value)
                {
                    return;
                }

                var oldValue = _alignmentEnergyString;
                _alignmentEnergyString = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(AlignmentEnergyStringPropertyName);
            }
        }

        #endregion

        #region IsTrackingModel

        /// <summary>
        /// The <see cref="IsTrackingModel" /> property's name.
        /// </summary>
        public const string IsTrackingModelPropertyName = "IsTrackingModel";

        private bool _isTrackingModel = true;

        /// <summary>
        /// Gets the IsTrackingModel property.
        /// </summary>
        public bool IsTrackingModel
        {
            get
            {
                return _isTrackingModel;
            }

            set
            {
                if (_isTrackingModel == value)
                {
                    return;
                }

                var oldValue = _isTrackingModel;
                _isTrackingModel = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(IsTrackingModelPropertyName);
            }
        }

        #endregion

        #region IsIntegrationPaused

        /// <summary>
        /// The <see cref="IsIntegrationPaused" /> property's name.
        /// </summary>
        public const string IsIntegrationPausedPropertyName = "IsIntegrationPaused";

        private bool _isIntegrationPaused = false;

        /// <summary>
        /// Gets the IsIntegrationPaused property.
        /// </summary>
        public bool IsIntegrationPaused
        {
            get
            {
                return _isIntegrationPaused;
            }

            set
            {
                if (_isIntegrationPaused == value)
                {
                    return;
                }

                var oldValue = _isIntegrationPaused;
                _isIntegrationPaused = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(IsIntegrationPausedPropertyName);
            }
        }

        #endregion

        #region IsTracking

        /// <summary>
        /// The <see cref="IsTracking" /> property's name.
        /// </summary>
        public const string IsTrackingPropertyName = "IsTracking";

        private bool _isTracking = false;

        /// <summary>
        /// Gets the IsTracking property.
        /// </summary>
        public bool IsTracking
        {
            get
            {
                return _isTracking;
            }

            set
            {
                if (_isTracking == value)
                {
                    return;
                }

                var oldValue = _isTracking;
                _isTracking = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(IsTrackingPropertyName);
            }
        }

        #endregion

        #region ShowRGBOverlay

        /// <summary>
        /// The <see cref="ShowRGBOverlay" /> property's name.
        /// </summary>
        public const string ShowRGBOverlayPropertyName = "ShowRGBOverlay";

        private bool _showRGBOverlay = false;

        /// <summary>
        /// Gets the ShowRGBOverlay property.
        /// </summary>
        public bool ShowRGBOverlay
        {
            get
            {
                return _showRGBOverlay;
            }

            set
            {
                if (_showRGBOverlay == value)
                {
                    return;
                }

                var oldValue = _showRGBOverlay;
                _showRGBOverlay = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(ShowRGBOverlayPropertyName);
            }
        }

        #endregion

        #region ZOffset

        /// <summary>
        /// The <see cref="ZOffset" /> property's name.
        /// </summary>
        public const string ZOffsetPropertyName = "ZOffset";

        private double _zOffset = 1.3;

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

                FaceTrackingVM.Kinect = _kinectSensor;

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

        #region ResidualImage

        /// <summary>
        /// The <see cref="ResidualImage" /> property's name.
        /// </summary>
        public const string ResidualImagePropertyName = "ResidualImage";

        private ImageSource _residualImage = null;

        /// <summary>
        /// Gets the ResidualImage property.
        /// </summary>
        public ImageSource ResidualImage
        {
            get
            {
                return _residualImage;
            }

            set
            {
                if (_residualImage == value)
                {
                    return;
                }

                var oldValue = _residualImage;
                _residualImage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(ResidualImagePropertyName);
            }
        }

        #endregion

        #region Fusion2OutputImage

        /// <summary>
        /// The <see cref="Fusion2OutputImage" /> property's name.
        /// </summary>
        public const string Fusion2OutputImagePropertyName = "Fusion2OutputImage";

        private ImageSource _fusion2OutputImage = null;

        /// <summary>
        /// Gets the Fusion2OutputImage property.
        /// </summary>
        public ImageSource Fusion2OutputImage
        {
            get
            {
                return _fusion2OutputImage;
            }

            set
            {
                if (_fusion2OutputImage == value)
                {
                    return;
                }

                var oldValue = _fusion2OutputImage;
                _fusion2OutputImage = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(Fusion2OutputImagePropertyName);
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

        #region FaceTrackingVM

        /// <summary>
        /// The <see cref="FaceTrackingVM" /> property's name.
        /// </summary>
        public const string FaceTrackingVMPropertyName = "FaceTrackingVM";

        private FaceTrackingViewModel _faceTrackingVM = new FaceTrackingViewModel();

        /// <summary>
        /// Gets the FaceTrackingVM property.
        /// </summary>
        public FaceTrackingViewModel FaceTrackingVM
        {
            get
            {
                return _faceTrackingVM;
            }

            set
            {
                if (_faceTrackingVM == value)
                {
                    return;
                }

                var oldValue = _faceTrackingVM;
                _faceTrackingVM = value;

                // Update bindings, no broadcast
                RaisePropertyChanged(FaceTrackingVMPropertyName);
            }
        }

        #endregion

        #endregion

        #region Constructors

        public MainViewModel()
        {
            if (IsInDesignMode)
            {
                // Code runs in Blend --> create design time data.
            }
            else
            {
                // Code runs "for real": Connect to service, etc...
                Init();
            }
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

                if (_fusionWorkQueue != null)
                {
                    _fusionWorkQueue.Dispose();
                    _fusionWorkQueue = null;
                }

                if (_kinectWorkQueue != null)
                {
                    _kinectWorkQueue.Dispose();
                    _kinectWorkQueue = null;
                }

                if (null != this.depthFloatBuffer)
                {
                    this.depthFloatBuffer.Dispose();
                }

                if (null != this.residualFloatBuffer)
                {
                    this.residualFloatBuffer.Dispose();
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

        #region Initialization

        private void Init()
        {

            _currentKinectFormat = new KinectFormat()
            {
                ColorImageFormat = ColorImageFormat.Undefined,
                DepthImageFormat = DepthImageFormat.Undefined,
                NumSkeletons = 0
            };

            _fusionWorkItemPool = new Pool<FusionWorkItem, DepthImageFormat>(5, _currentKinectFormat.DepthImageFormat, FusionWorkItem.Create);

            _fusionWorkQueue = new WorkQueue<FusionWorkItem>(ProcessFusionFrameBackground)
            {
                CanceledCallback = ReturnFusionWorkItem,
                MaxQueueLength = 2
            };

            _kinectFrameWorkItemPool = new Pool<KinectFrameWorkItem, KinectFormat>(5, _currentKinectFormat, KinectFrameWorkItem.Create);

            _kinectWorkQueue = new WorkQueue<KinectFrameWorkItem>(ProcessKinectFrame)
            {
                CanceledCallback = ReturnKinectFrameWorkItem,
                MaxQueueLength = 1
            };

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
                newSensor.ColorStream.Enable(DefaultColorImageFormat);
                newSensor.DepthStream.Enable(DefaultDepthImageFormat);
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

                var smoothParams = new TransformSmoothParameters()
                {
                    Smoothing = 0.8f,
                    Correction = 0.2f,
                    Prediction = 0.5f,
                    JitterRadius = 0.10f,
                    MaxDeviationRadius = 0.04f
                };

                newSensor.SkeletonStream.Enable(smoothParams);
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

        #endregion

        #region Kinect

        private void KinectSensorOnAllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            bool formatChanged = false;

            ColorImageFrame colorImageFrame = null;
            DepthImageFrame depthImageFrame = null;
            SkeletonFrame skeletonFrame = null;
            try
            {
                colorImageFrame = e.OpenColorImageFrame();
                depthImageFrame = e.OpenDepthImageFrame();

                if (colorImageFrame != null)
                {
                    if (_currentKinectFormat.ColorImageFormat != colorImageFrame.Format)
                    {
                        formatChanged = true;
                        _currentKinectFormat.ColorImageFormat = colorImageFrame.Format;
                    }
                }

                if (depthImageFrame != null)
                {
                    if (_currentKinectFormat.DepthImageFormat != depthImageFrame.Format)
                    {
                        formatChanged = true;

                        _currentKinectFormat.DepthImageFormat = depthImageFrame.Format;

                        _fusionWorkItemPool.Format = depthImageFrame.Format;

                        var depthWidth = depthImageFrame.Width;
                        var depthHeight = depthImageFrame.Height;

                        FaceTrackingVM.DepthWidth = depthWidth;
                        FaceTrackingVM.DepthHeight = depthHeight;

                        this._depthImageData = new byte[depthImageFrame.PixelDataLength * 4];
                        this._modDepthImageData = new byte[depthImageFrame.PixelDataLength * 4];
                        this._mappedColorImageData = new byte[depthImageFrame.PixelDataLength * 4];

                        this._depthImageWritableBitmap = new WriteableBitmap(
                            depthWidth, depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        this._modDepthImageWritableBitmap = new WriteableBitmap(
                            depthWidth, depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        _colorImageWritableBitmap = new WriteableBitmap(
                            depthWidth, depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        _residualImageData = new byte[depthImageFrame.PixelDataLength * 4];
                        _residualWritableBitmap = new WriteableBitmap(
                            depthWidth, depthHeight, 96, 96, PixelFormats.Bgr32, null);

                        ResidualImage = _residualWritableBitmap;
                        ColorImage = _colorImageWritableBitmap;
                        DepthImage = _depthImageWritableBitmap;
                        FusionInputImage = _modDepthImageWritableBitmap;
                    }

                }

                skeletonFrame = e.OpenSkeletonFrame();
                if (skeletonFrame != null)
                {
                    if (_currentKinectFormat.NumSkeletons != skeletonFrame.SkeletonArrayLength)
                    {
                        _currentKinectFormat.NumSkeletons = skeletonFrame.SkeletonArrayLength;
                        formatChanged = true;
                    }
                }

                if (formatChanged)
                {
                    _kinectFrameWorkItemPool.Format = _currentKinectFormat;
                }

                if (colorImageFrame != null &&
                    depthImageFrame != null &&
                    skeletonFrame != null)
                {
                    var workItem = _kinectFrameWorkItemPool.Pop();

                    workItem.FrameNumber = depthImageFrame.FrameNumber;

                    colorImageFrame.CopyPixelDataTo(workItem.ColorPixels);
                    depthImageFrame.CopyDepthImagePixelDataTo(workItem.DepthImagePixels);
                    skeletonFrame.CopySkeletonDataTo(workItem.Skeletons);

                    var mapper = KinectSensor.CoordinateMapper;

                    mapper.MapColorFrameToDepthFrame(workItem.Format.ColorImageFormat,
                                                     workItem.Format.DepthImageFormat,
                                                     workItem.DepthImagePixels,
                                                     workItem.ColorMappedToDepthPoints);

                    if (_kinectWorkQueue != null)
                    {
                        _kinectWorkQueue.AddWork(workItem);
                    }
                }

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

        private void ReturnKinectFrameWorkItem(KinectFrameWorkItem workItem)
        {
            _kinectFrameWorkItemPool.Push(workItem);
        }

        private void ProcessKinectFrame(KinectFrameWorkItem workItem)
        {
            ProcessSkeletonFrame(workItem);
            ProcessColorFrame(workItem);
            ProcessDepthFrame(workItem);

            ProcessFusionFrame(workItem.DepthImagePixels);
            //var skeletonList = new List<Skeleton>();
            //if (_activeSkeleton != null)
            //{
            //    skeletonList.Add(_activeSkeleton);
            //}

            //FaceTrackingVM.TrackFrame(DefaultColorImageFormat,
            //                         _colorImageData,
            //                         DefaultDepthImageFormat,
            //                         _depthImagePixels,
            //                         skeletonList,
            //                         workItem.FrameNumber);

            rawFrameCount++;

            _kinectFrameWorkItemPool.Push(workItem);

            _syncContext.Post((SendOrPostCallback)UpdateKinectFrameUI, null);
        }

        private void UpdateKinectFrameUI(object state)
        {
            _depthImageWritableBitmap.WritePixels(
                new Int32Rect(0, 0, _depthImageWritableBitmap.PixelWidth, _depthImageWritableBitmap.PixelHeight),
                _depthImageData,
                _depthImageWritableBitmap.PixelWidth * 4,
                0);

            _colorImageWritableBitmap.WritePixels(
                new Int32Rect(0, 0, _colorImageWritableBitmap.PixelWidth, _colorImageWritableBitmap.PixelHeight),
                _mappedColorImageData,
                _colorImageWritableBitmap.PixelWidth * 4,
                0);

            SkeletonJointVM.ProcessFrame(KinectSensor.CoordinateMapper, _activeSkeleton, _currentKinectFormat.DepthImageFormat);

            CheckFPS();
        }

        private void ProcessSkeletonFrame(KinectFrameWorkItem workItem)
        {
            var skeletonList = workItem.Skeletons.ToList();
            var closestSkeleton = skeletonList.Where(s => s.TrackingState == SkeletonTrackingState.Tracked)
                                              .OrderBy(s => s.Position.Z * Math.Abs(s.Position.X))
                                              .FirstOrDefault();

            bool newSkeleton = false;
            if (closestSkeleton != null &&
                (_activeSkeleton == null ||
                 _activeSkeleton.TrackingId != closestSkeleton.TrackingId))
            {
                newSkeleton = true;
            }

            if (closestSkeleton == null)
            {
                _activeSkeletonLostCount++;

                if (_activeSkeletonLostCount > _activeSkeletonLostLimit)
                {
                    _activeSkeleton = null;
                }
                _activeSkeletonId = InactiveSkeletonId;
            }
            else
            {
                _activeSkeletonLostCount = 0;
                _activeSkeleton = closestSkeleton;
                _activeSkeletonId = skeletonList.IndexOf(closestSkeleton) + 1;


                var headJoint = closestSkeleton.Joints[JointType.Head];
                var neckJoint = closestSkeleton.Joints[JointType.ShoulderCenter];
                float headFraction = 0.85f;
                _volumeCenter.X = (headJoint.Position.X * headFraction) + (neckJoint.Position.X) * (1.0f - headFraction);
                _volumeCenter.Y = (headJoint.Position.Y * headFraction) + (neckJoint.Position.Y) * (1.0f - headFraction);
                _volumeCenter.Z = (headJoint.Position.Z * headFraction) + (neckJoint.Position.Z) * (1.0f - headFraction);
            }

            if (newSkeleton)
            {
                ResetReconstruction();
            }
        }

        private unsafe void ProcessDepthFrame(KinectFrameWorkItem workItem)
        {
            var depthSize = FormatHelper.GetDepthSize(workItem.Format.DepthImageFormat);
            int width = (int)depthSize.Width;
            int height = (int)depthSize.Height;

            double maxDepth = 4000;
            double minDepth = 400;

            fixed (byte* depthPtrFixed = _depthImageData)
            {
                fixed (DepthImagePixel* pixelPtrFixed = workItem.DepthImagePixels)
                {
                    int* depthIntPtr = (int*)depthPtrFixed;
                    DepthImagePixel* pixelPtr = pixelPtrFixed;

                    int len = width * height;

                    for (int y = 0; y < height; y++)
                    {
                        for (int x = 0; x < width; x++)
                        {
                            int srcIndex = (width - 1 - x) + y * width;
                            int targetIndex = x + y * width;

                            var dip = *(pixelPtr + srcIndex);

                            short depth = dip.Depth;
                            int playerIndex = dip.PlayerIndex;

                            byte value = (byte)(255 - 255 * (depth - minDepth) / maxDepth);

                            if (depth <= 0)
                            {
                                value = 0;
                            }

                            int shiftValue = ((255 << 24) | (value << 16) | (value << 8) | (value));

                            *depthIntPtr = shiftValue;
                            depthIntPtr++;

                        }
                    }
                }
            }
        }

        #region Old
        /*
        private unsafe void ProcessDepthFrameMod()
        {
            int width = _depthWidth;
            int height = _depthHeight;

            //Array.Clear(_modDepthImagePixels, 0, _modDepthImagePixels.Length);

            //short defaultDepth = (short)KinectSensor.DepthStream.UnknownDepth;
            //var defaultDIP = new DepthImagePixel() { Depth = defaultDepth };

            double maxDepth = 4000;
            double minDepth = 400;

            //bool processModImage = _activeSkeleton != null;

            //double hx = 0, hy = 0, hzMax = 0, hzMin = 0;
            //double headNeckDist2 = 0;
            //double headDepthThreshold = 300;

            //if (processModImage)
            //{
            //    var mapper = _kinectSensor.CoordinateMapper;

            //    var headJoint = _activeSkeleton.Joints[JointType.Head];
            //    var neckJoint = _activeSkeleton.Joints[JointType.ShoulderCenter];
            //    if (headJoint.TrackingState == JointTrackingState.Tracked &&
            //        neckJoint.TrackingState == JointTrackingState.Tracked)
            //    {
            //        var headPoint = mapper.MapSkeletonPointToDepthPoint(headJoint.Position, DefaultDepthImageFormat);
            //        var pos = new SkeletonPoint()
            //            {
            //                X = headJoint.Position.X,
            //                Y = headJoint.Position.Y - 0.200f,
            //                Z = headJoint.Position.Z
            //            };
            //        var neckPoint = mapper.MapSkeletonPointToDepthPoint(pos, DefaultDepthImageFormat);

            //        hx = depthWidth - headPoint.X;
            //        hy = headPoint.Y;
            //        hzMax = headPoint.Depth + headDepthThreshold;
            //        hzMin = headPoint.Depth - headDepthThreshold;
            //        double factor = 2;
            //        headNeckDist2 = (Math.Pow(neckPoint.X - headPoint.X, 2) +
            //                         Math.Pow(neckPoint.Y - headPoint.Y, 2)) * factor * factor;
            //    }
            //    else
            //    {
            //        processModImage = false;
            //    }
            //}

            fixed (byte* depthPtrFixed = _depthImageData)//, modDepthPtrFixed = _modDepthImageData)
            {
                int* depthIntPtr = (int*)depthPtrFixed;
                //int* modDepthIntPtr = (int*)modDepthPtrFixed;

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

                        //byte modValue = 0;
                        //byte modRMult = 0;

                        ////if (processModImage &&
                        ////    playerIndex == _activeSkeletonId)
                        //{
                        //    var dx = x - hx;
                        //    var dy = y - hy;
                        //    var dist2 = dx * dx + dy * dy;

                        //    modValue = value;

                        //    //if (dist2 < headNeckDist2 &&
                        //    //    depth >= hzMin &&
                        //    //    depth <= hzMax)
                        //    {
                        //        modRMult = 1;
                        //        _modDepthImagePixels[srcIndex] = new DepthImagePixel() { Depth = depth };
                        //    }
                        //}
                        //else
                        //{
                        //    _modDepthImagePixels[targetIndex] = defaultDIP;
                        //}

                        if (depth <= 0)
                        {
                            value = 0;
                        }

                        //_depthImageData[targetIndex * 4 + 0] = value;
                        //_depthImageData[targetIndex * 4 + 1] = value;
                        //_depthImageData[targetIndex * 4 + 2] = value;

                        int shiftValue = ((255 << 24) | (value << 16) | (value << 8) | (value));

                        *depthIntPtr = shiftValue;
                        depthIntPtr++;

                        //_modDepthImageData[targetIndex * 4 + 0] = modValue;
                        //_modDepthImageData[targetIndex * 4 + 1] = modValue;
                        //_modDepthImageData[targetIndex * 4 + 2] = modValue;

                        //int shiftModValue = ((255 << 24) | (modValue << 16) | ((modValue * modRMult) << 8) | ((modValue * modRMult)));

                        //*modDepthIntPtr = shiftModValue;
                        //modDepthIntPtr++;

                    }
                }
            }

            _depthImageWritableBitmap.WritePixels(
                new Int32Rect(0, 0, width, height),
                _depthImageData,
                width * 4,
                0);

            //_modDepthImageWritableBitmap.WritePixels(
            //    new Int32Rect(0, 0, width, height),
            //    _modDepthImageData,
            //    width * 4,
            //    0);

            //if (UserFusionOnly)
            //{
            //    ProcessFusionFrame((DepthImagePixel[])_modDepthImagePixels.Clone());
            //}
            //else
            //{
            //}
            ProcessFusionFrame((DepthImagePixel[])_depthImagePixels.Clone());
        }
        */
        #endregion

        private unsafe void ProcessColorFrame(KinectFrameWorkItem workItem)
        {
            var depthSize = FormatHelper.GetDepthSize(workItem.Format.DepthImageFormat);
            var colorSize = FormatHelper.GetColorSize(workItem.Format.ColorImageFormat);

            int colorWidth = (int)colorSize.Width;
            int colorHeight = (int)colorSize.Height;
            int depthWidth = (int)depthSize.Width;
            int depthHeight = (int)depthSize.Height;

            Array.Clear(_mappedColorImageData, 0, _mappedColorImageData.Length);

            var map = workItem.ColorMappedToDepthPoints;

            int depthWidthMinusOne = depthWidth - 1;

            fixed (byte* colorPtrFixed = workItem.ColorPixels, mappedColorPtrFixed = _mappedColorImageData)
            {
                fixed (DepthImagePoint* mapPtrFixed = workItem.ColorMappedToDepthPoints)
                {
                    int* colorIntPtr = (int*)colorPtrFixed;
                    int* mappedColorIntPtr = (int*)mappedColorPtrFixed;
                    DepthImagePoint* mapPtr = mapPtrFixed;

                    //for (int y = 0; y < colorHeight; y += 1)
                    //{
                    //    for (int x = 0; x < colorWidth; x += 1)
                    //    {
                    //        int srcIndex = x + y * colorWidth;
                    int len = colorWidth * colorHeight;
                    for (int i = 0; i < len; i++)
                    {
                        var coord = *mapPtr;

                        int cx = coord.X;
                        int cy = coord.Y;
                        if (cx >= 0 && cx < depthWidth &&
                            cy >= 0 && cy < depthHeight)
                        {
                            int targetIndex = (depthWidthMinusOne - cx) + cy * depthWidth;

                            *(mappedColorIntPtr + targetIndex) = *(colorIntPtr);
                        }

                        mapPtr++;
                        colorIntPtr++;
                    }
                    //    }
                    //}
                }
            }

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
        /// Update the FPS reading in the status text bar
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckFPS()
        {
            DateTime now = DateTime.Now;
            var span = now - _lastFPSUpdate;
            if (span.TotalSeconds < 1.0)
                return;

            _lastFPSUpdate = now;

            double fusionFPS = this.processedFrameCount / span.TotalSeconds;
            double rawFPS = this.rawFrameCount / span.TotalSeconds;

            // Update the FPS reading

            StatusMessage = String.Format("Kinect FPS: {0} Fusion FPS: {1}", rawFPS.ToString("F1"), fusionFPS.ToString("F1"));

            // Reset the frame count
            this.processedFrameCount = 0;
            this.rawFrameCount = 0;
        }

        #endregion

        #region Fusion

        private void InitFusion()
        {
            if (_isFusionInitialized)
                return;

            _isFusionInitialized = true;

            this.frameDataLength = KinectSensor.DepthStream.FramePixelDataLength;

            // Allocate space to put the color pixels we'll create
            this.colorPixels = new int[(int)(DepthImageSize.Width * 2 * DepthImageSize.Height * 2)];

            // This is the bitmap we'll display on-screen
            this.colorFusionBitmap = new WriteableBitmap(
                (int)DepthImageSize.Width * 2,
                (int)DepthImageSize.Height * 2,
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
            catch (ArgumentException)
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
            this.depthFloatBuffer = new FusionFloatImageFrame((int)DepthImageSize.Width, (int)DepthImageSize.Height);
            this.residualFloatBuffer = new FusionFloatImageFrame((int)DepthImageSize.Width, (int)DepthImageSize.Height);
            _residualData = new float[(int)(DepthImageSize.Width * DepthImageSize.Height)];

            // Point cloud frames generated from the depth float input
            this.pointCloudBuffer = new FusionPointCloudImageFrame((int)DepthImageSize.Width * 2, (int)DepthImageSize.Height * 2);

            // Create images to raycast the Reconstruction Volume
            this.shadedSurfaceColorFrame = new FusionColorImageFrame((int)DepthImageSize.Width * 2, (int)DepthImageSize.Height * 2);

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
            _currentRotationDegrees = 0;

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
                    double volumeSizeZ = (VoxelResolutionZ / (double)VoxelsPerMeter);
                    if (!UserFusionOnly)
                    {
                        _volumeCenter.X = 0;
                        _volumeCenter.Y = 0;
                        _volumeCenter.Z = (float)(ZOffset);
                    }
                    worldToVolumeTransform.M41 += (float)(_volumeCenter.X * VoxelsPerMeter);
                    worldToVolumeTransform.M42 += (float)(_volumeCenter.Y * VoxelsPerMeter);
                    worldToVolumeTransform.M43 -= (float)((_volumeCenter.Z - 0.5 * volumeSizeZ) * VoxelsPerMeter);

                    _currentVolumeCenter = _volumeCenter;
                    Trace.WriteLine("Reset reconstruction at center: " + _volumeCenter.X + ", " + _volumeCenter.Y + " " + _volumeCenter.Z);

                    this.volume.ResetReconstruction(this.worldToCameraTransform, worldToVolumeTransform);
                }
                else
                {
                    this.volume.ResetReconstruction(this.worldToCameraTransform);
                }
            }

            this.rawFrameCount = 0;
            this.processedFrameCount = 0;
        }

        private void ProcessFusionFrame(DepthImagePixel[] depthPixels)
        {
            var workItem = _fusionWorkItemPool.Pop();

            if (workItem == null)
            {
                Trace.WriteLine("Fusion Depth Pool empty");
                return;
            }

            Array.Copy(depthPixels, workItem.Data, depthPixels.Length);

            if (_fusionWorkQueue != null)
            {
                _fusionWorkQueue.AddWork(workItem);
            }
        }

        private void FusionUpdateUI(object state)
        {
            bool lastTrackSucceeded = (bool)state;

            this.IsTracking = lastTrackSucceeded;

            // Write the pixel data into our bitmap
            colorFusionBitmap.WritePixels(
                new Int32Rect(0, 0, this.colorFusionBitmap.PixelWidth, this.colorFusionBitmap.PixelHeight),
                this.colorPixels,
                this.colorFusionBitmap.PixelWidth * sizeof(int),
                0);

            _residualWritableBitmap.WritePixels(new Int32Rect(0, 0, _residualWritableBitmap.PixelWidth, _residualWritableBitmap.PixelHeight),
                                                _residualImageData, _residualWritableBitmap.PixelWidth * sizeof(int), 0);

            this.AlignmentEnergyString = "Alignment Energy: " + _alignmentEnergy.ToString("F6");
        }

        private void ReturnFusionWorkItem(FusionWorkItem workItem)
        {
            _fusionWorkItemPool.Push(workItem);
        }

        /// <summary>
        /// Process the depth input
        /// </summary>
        /// <param name="depthPixels">The depth data array to be processed</param>
        private void ProcessFusionFrameBackground(FusionWorkItem workItem)
        {
            Debug.Assert(null != this.volume, "volume should be initialized");
            Debug.Assert(null != this.shadedSurfaceColorFrame, "shaded surface should be initialized");
            Debug.Assert(null != this.colorFusionBitmap, "color bitmap should be initialized");

            try
            {
                DepthImagePixel[] depthPixels = workItem.Data;
                bool trackingSucceeded = TrackIntegrate(depthPixels);

                if (processedFrameCount % 2 == 0)
                {
                    RenderFusion();
                }
                // The input frame was processed successfully, increase the processed frame count
                ++this.processedFrameCount;

                _fusionWorkItemPool.Push(workItem);

                _syncContext.Post((SendOrPostCallback)FusionUpdateUI, trackingSucceeded);
                //return trackingSucceeded;
            }
            catch (InvalidOperationException ex)
            {
                StatusMessage = ex.Message;
                //return false;
            }
            finally
            {
            }
        }

        private bool TrackIntegrate(DepthImagePixel[] depthPixels)
        {
            // Convert the depth image frame to depth float image frame
            FusionDepthProcessor.DepthToDepthFloatFrame(
                depthPixels,
                (int)DepthImageSize.Width,
                (int)DepthImageSize.Height,
                this.depthFloatBuffer,
                FusionDepthProcessor.DefaultMinimumDepth,
                FusionDepthProcessor.DefaultMaximumDepth,
                false);

            bool trackingSucceeded = this.volume.AlignDepthFloatToReconstruction(
                    depthFloatBuffer,
                    FusionDepthProcessor.DefaultAlignIterationCount,
                    null,//residualFloatBuffer,
                    out _alignmentEnergy,
                    volume.GetCurrentWorldToCameraTransform());

            //ProcessResidualImage();

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
                this.worldToCameraTransform = volume.GetCurrentWorldToCameraTransform();

                if (!IsIntegrationPaused)
                {
                    this.volume.IntegrateFrame(depthFloatBuffer, IntegrationWeight, this.worldToCameraTransform);
                }

                this.trackingErrorCount = 0;
            }

            if (AutoResetReconstructionWhenLost && !trackingSucceeded && this.trackingErrorCount == MaxTrackingErrors)
            {
                // Auto Reset due to bad tracking
                StatusMessage = Properties.Resources.ResetVolume;

                // Automatically Clear Volume and reset tracking if tracking fails
                this.ResetReconstruction();
            }
            return trackingSucceeded;
        }

        private void RenderFusion()
        {
            Matrix3D m = Matrix3D.Identity;
            m = worldToCameraTransform.ToMatrix3D();

            _currentRotationDegrees += _rotationRateInDegrees;

            double zSize = VoxelResolutionZ / (double)VoxelsPerMeter;
            m.Translate(new Vector3D(_currentVolumeCenter.X,
                                     _currentVolumeCenter.Y,
                                     -_currentVolumeCenter.Z));
            m.Rotate(new Quaternion(new Vector3D(0, 1, 0), _currentRotationDegrees));

            double zDelta = _volumeCenter.Z - _currentVolumeCenter.Z;

            m.Translate(new Vector3D(0,
                                    0,
                                    1.75 * zSize));


            //m.Translate(new Vector3D(0 * VoxelsPerMeter,
            //                        0,
            //                        -1.0 * (ZOffset + 0.5 * zSize)));
            //m.Translate(new Vector3D(_currentVolumeCenter.X, _currentVolumeCenter.Y, _currentVolumeCenter.Z + zSize));

            var cameraTransform = m.ToMatrix4();

            var viewCam = cameraTransform;

            if (!IsTrackingModel)
            {
                viewCam = worldToCameraTransform;
            }

            // Calculate the point cloud
            this.volume.CalculatePointCloud(this.pointCloudBuffer, viewCam);

            float volSizeX = VoxelResolutionX / (float)VoxelsPerMeter;
            float volSizeY = VoxelResolutionY / (float)VoxelsPerMeter;
            float volSizeZ = VoxelResolutionZ / (float)VoxelsPerMeter;

            Matrix4 worldToBGRTransform = Matrix4.Identity;
            worldToBGRTransform.M11 = VoxelsPerMeter / (float)VoxelResolutionX;
            worldToBGRTransform.M22 = VoxelsPerMeter / (float)VoxelResolutionY;
            worldToBGRTransform.M33 = VoxelsPerMeter / (float)VoxelResolutionZ;
            worldToBGRTransform.M41 = -_currentVolumeCenter.X - 0.5f * volSizeX;
            worldToBGRTransform.M42 = _currentVolumeCenter.Y - 0.5f * volSizeY;
            worldToBGRTransform.M43 = _currentVolumeCenter.Z - 0.5f * volSizeZ;
            worldToBGRTransform.M44 = 1.0f;

            // Shade point cloud and render
            FusionDepthProcessor.ShadePointCloud(
                this.pointCloudBuffer,
                viewCam,
                worldToBGRTransform,
                null,
                this.shadedSurfaceColorFrame);

            this.shadedSurfaceColorFrame.CopyPixelDataTo(this.colorPixels);
        }

        private void ProcessResidualImage()
        {
            residualFloatBuffer.CopyPixelDataTo(_residualData);

            int len = _residualData.Length;

            for (int i = 0; i < len; i++)
            {
                float data = _residualData[i];

                if (data <= 1.0)
                {
                    _residualImageData[i * 4 + 0] = (byte)(255 * MathUtility.Clamp(1 - data, 0, 1));
                    _residualImageData[i * 4 + 1] = (byte)(255 * MathUtility.Clamp(1 - Math.Abs(data), 0, 1));
                    _residualImageData[i * 4 + 2] = (byte)(255 * MathUtility.Clamp(1 + data, 0, 1));
                }
                else
                {
                    _residualImageData[i * 4 + 0] = 0;
                    _residualImageData[i * 4 + 1] = 0;
                    _residualImageData[i * 4 + 2] = 0;
                }
            }

        }

        #endregion

        #endregion
    }
}