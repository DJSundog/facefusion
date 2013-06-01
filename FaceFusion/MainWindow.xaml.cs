//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (updateMatrix) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace FaceFusion
{
    using System;
    using System.Diagnostics;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Threading;
    using Microsoft.Kinect;
    using Microsoft.Kinect.Toolkit.Fusion;
using FaceFusion.ViewModels;
using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        MainViewModel _mainViewModel;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            this.InitializeComponent();

            this.Loaded += new RoutedEventHandler(MainWindow_Loaded);
            Application.Current.Exit += new ExitEventHandler(Current_Exit);
        }

        void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            _mainViewModel = new MainViewModel();
            this.DataContext = _mainViewModel;
        }

        void Current_Exit(object sender, ExitEventArgs e)
        {
            if (_mainViewModel != null)
            {
                _mainViewModel.Dispose();
            }
        }
    }
}
