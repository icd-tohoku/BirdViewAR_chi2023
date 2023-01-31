using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices.WindowsRuntime;
using Windows.Foundation;
using Windows.Foundation.Collections;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Data;
using Windows.UI.Xaml.Input;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Navigation;
using DJI.WindowsSDK;
using Windows.UI.Xaml.Media.Imaging;
using DJIVideoParser;
using System.Threading.Tasks;
using System.Net;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
using System.Text;
using Windows.Networking;
using Windows.Gaming.Input;
using Windows.Storage;
using Windows.Graphics.Imaging;
//using OpenCVBridge;
using Windows.UI.Core;
using Windows.System;
using Windows.UI.ViewManagement;
using Windows.UI.Core.Preview;
// The Blank Page item template is documented at https://go.microsoft.com/fwlink/?LinkId=402352&clcid=0x409

namespace DJIWSDKDemo
{
    /// <summary>
    /// An empty page that can be used on its own or navigated to within a Frame.
    /// </summary>
    public sealed partial class MainPage : Page
    {
        private DJIVideoParser.Parser videoParser;


        DatagramSocket socket = new DatagramSocket();
        int port = 3333;
        
        private readonly object myLock = new object();
        private List<Gamepad> myConrollers = new List<Gamepad>();


        private DispatcherTimer _timer;

        LocationCoordinate2D? _currentLocation;
        ResultValue<DoubleMsg?> _currentAltitude;

        StorageFolder _stFolder;
        StorageFile _stFile;
        StorageFile _recFile;
        string _writeData;
        bool _isWitten;

        private int _taskCounter;
        private bool _isRecord;
        private int _arMode;

        int _isGround;

        public MainPage()
        {
            this.InitializeComponent();

            _isWitten = false;
            _isRecord = false;
            _isGround = 1;
            _arMode = 0;
            ElementSoundPlayer.State = ElementSoundPlayerState.On;
  

            _taskCounter = 0;
            DJISDKManager.Instance.SDKRegistrationStateChanged += Instance_SDKRegistrationEvent;
            //Replace app key with the real key registered. Make sure that the key is matched with your application's package id.
            DJISDKManager.Instance.RegisterApp("0170a33cb2e5f47c3c979c6a");
            Window.Current.CoreWindow.KeyDown += mp_KeyDown;
        }

        private async void mp_KeyDown(CoreWindow sender, KeyEventArgs args)
        {
            //System.Diagnostics.Debug.WriteLine(args.VirtualKey);
            if(args.VirtualKey == VirtualKey.I)
            {
                System.Diagnostics.Debug.WriteLine(args.VirtualKey);
                
                var currentLocation = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAircraftLocationAsync();
                double nowLat = currentLocation.value.Value.latitude;
                double nowLang = currentLocation.value.Value.longitude;
                var currentAltitude = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAltitudeAsync();
                double nowAltitude = currentAltitude.value.Value.value;
                var rotation = (await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAttitudeAsync()).value.Value.yaw;
                System.Diagnostics.Debug.WriteLine("Lat:" + nowLat.ToString());
                System.Diagnostics.Debug.WriteLine("Lng:" + nowLang.ToString());
                System.Diagnostics.Debug.WriteLine("Rot:" + rotation.ToString());
                System.Diagnostics.Debug.WriteLine("Alt:" + nowAltitude.ToString());
                
            }else if(args.VirtualKey == VirtualKey.S)
            {

                var view = ApplicationView.GetForCurrentView();
                if (_isRecord)
                {
                    view.ExitFullScreenMode();
                    var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StopRecordAsync();
                    _isRecord = false;
                }
                else
                {
                    view.TryEnterFullScreenMode();
                    var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartRecordAsync();
                    _isRecord = true;
                }

            }
            else if(args.VirtualKey == VirtualKey.R)
            {
                var dtime = System.DateTime.Now;

                ElementSoundPlayer.Play(ElementSoundKind.Show);
                System.Diagnostics.Debug.WriteLine("sound");  
                string crrrTime = dtime.Year.ToString() + "-" + dtime.Month.ToString() + "-" + dtime.Day.ToString() + " " + dtime.Hour.ToString() + ":" + dtime.Minute.ToString() + ":" + dtime.Second.ToString() + "." + dtime.Millisecond.ToString() + "\r\n";
                //_writeData = _writeData +crrrTime+","+ nowLat.ToString() + "," + nowLang.ToString() + "," + nowAltitude.ToString() + "," + leftStickX.ToString() + "," + leftStickY.ToString() + "," + rightStikX.ToString() + "," + rightStikY.ToString() + "," + modVelocityX.ToString() + "," + modVelocityY.ToString() + "," + velocityZ.ToString() + "," + rotation.ToString() + "\r\n";
                //await FileIO.WriteTextAsync(_stFile, _writeData);
                 //var writeData = crrrTime;
                await FileIO.AppendTextAsync(_recFile, crrrTime);
            }else if(args.VirtualKey == VirtualKey.T)
            {
                await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
            }
        }

        //Callback of SDKRegistrationEvent
        private async void Instance_SDKRegistrationEvent(SDKRegistrationState state, SDKError resultCode)
        {
            if (resultCode == SDKError.NO_ERROR)
            {
                System.Diagnostics.Debug.WriteLine("Register app successfully.");
                var dtime = System.DateTime.Now;
                _stFolder = Windows.Storage.ApplicationData.Current.LocalFolder;
                string fname ="Log"+dtime.Year.ToString()+dtime.Month.ToString()+dtime.Day.ToString()+dtime.Hour.ToString() + dtime.Minute.ToString()+".txt";
                _stFile = await _stFolder.CreateFileAsync(fname,Windows.Storage.CreationCollisionOption.ReplaceExisting);
                string fname_r = "RecordTime" + dtime.Year.ToString() + dtime.Month.ToString() + dtime.Day.ToString() + dtime.Hour.ToString() + dtime.Minute.ToString() + ".txt";
                _recFile = await _stFolder.CreateFileAsync(fname_r, Windows.Storage.CreationCollisionOption.ReplaceExisting);
                //await FileIO.WriteTextAsync(_stFile, "test,test\r\ntest,test");
                //await FileIO.WriteTextAsync(_stFile, (dtime.Hour.ToString() + ":" + dtime.Minute.ToString()+":"+dtime.Second.ToString()));
                DJISDKManager.Instance.ComponentManager.GetProductHandler(0).ProductTypeChanged += async delegate (object sender, ProductTypeMsg? value)
                {
                    //System.Diagnostics.Debug.WriteLine("connecting");
                    await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () =>
                    {
                        if (value != null && value?.value != ProductType.UNRECOGNIZED)
                        {
                            System.Diagnostics.Debug.WriteLine("The Aircraft is connected now.");

                            //You can load/display your pages according to the aircraft connection state here.
                            _timer.Start();
                            SDKError enableErr = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).SetGroundStationModeEnabledAsync(new BoolMsg() { value = true });
                            //DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).AircraftLocationChanged += OnLocationChanged;
                            await Dispatcher.RunAsync(Windows.UI.Core.CoreDispatcherPriority.Normal, async () =>
                            {
                                //Raw data and decoded data listener
                                if (videoParser == null)
                                {
                                    videoParser = new DJIVideoParser.Parser();
                                    videoParser.Initialize(delegate (byte[] data)
                                    {
                                        //Note: This function must be called because we need DJI Windows SDK to help us to parse frame data.
                                        return DJISDKManager.Instance.VideoFeeder.ParseAssitantDecodingInfo(0, data);
                                    });
                                    //Set the swapChainPanel to display and set the decoded data callback.
                                    videoParser.SetSurfaceAndVideoCallback(0, 0, swapChainPanel, ReceiveDecodedData);
                                    DJISDKManager.Instance.VideoFeeder.GetPrimaryVideoFeed(0).VideoDataUpdated += OnVideoPush;
                                }
                                //get the camera type and observe the CameraTypeChanged event.
                                DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).CameraTypeChanged += OnCameraTypeChanged;
                                DJISDKManager.Instance.ComponentManager.GetRemoteControllerHandler(0, 0).RCCustomButton1DownChanged += C1Pushed;
                                DJISDKManager.Instance.ComponentManager.GetRemoteControllerHandler(0, 0).RCCustomButton2DownChanged += C2Pushed;

                                var type = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).GetCameraTypeAsync();
                                OnCameraTypeChanged(this, type.value);
                                
                                SetCameraWorkMode(CameraWorkMode.RECORD_VIDEO);
                                
                                var mltErr =  await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).SetMultipleFlightModeEnabledAsync(new BoolMsg() { value = true });

                            });

                            //Windows.UI.ViewManagement.ApplicationView.GetForCurrentView().TryEnterFullScreenMode();
                        }
                        else
                        {
                            System.Diagnostics.Debug.WriteLine("The Aircraft is disconnected now.");
                            //You can hide your pages according to the aircraft connection state here, or show the connection tips to the users.
                        }
                    });
                };



            }
            else
            {
                System.Diagnostics.Debug.WriteLine("SDK register failed, the error is: ");
                System.Diagnostics.Debug.WriteLine(resultCode.ToString());
            }
        }


        //raw data
        void OnVideoPush(VideoFeed sender, byte[] bytes)
        {
            videoParser.PushVideoData(0, 0, bytes, bytes.Length);
        }

        //Decode data. Do nothing here. This function would return a bytes array with image data in RGBA format.
        async void ReceiveDecodedData(byte[] data, int width, int height)
        {
 
        }

        //We need to set the camera type of the aircraft to the DJIVideoParser. After setting camera type, DJIVideoParser would correct the distortion of the video automatically.
        private void OnCameraTypeChanged(object sender, CameraTypeMsg? value)
        {
            if (value != null)
            {
                switch (value.Value.value)
                {
                    case CameraType.MAVIC_2_ZOOM:
                        this.videoParser.SetCameraSensor(AircraftCameraType.Mavic2Zoom);
                        break;
                    case CameraType.MAVIC_2_PRO:
                        this.videoParser.SetCameraSensor(AircraftCameraType.Mavic2Pro);
                        break;
                    default:
                        this.videoParser.SetCameraSensor(AircraftCameraType.Others);
                        break;
                }

            }
        }





        private async void Btn_TakeOff_Click(object sender, RoutedEventArgs e)
        {
            var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartTakeoffAsync();
            System.Diagnostics.Debug.WriteLine(res.ToString());
        }

        private async void Btn_AutoRanding_Click(object sender, RoutedEventArgs e)
        {
            var res = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).StartAutoLandingAsync();
            System.Diagnostics.Debug.WriteLine(res.ToString());
        }


        private async void WaypointmissionStateChanged(object sender, WaypointMissionStateTransition? value)
        {
            
            var errstate = DJISDKManager.Instance.WaypointMissionManager.GetWaypointMissionHandler(0).GetCurrentState();
            System.Diagnostics.Debug.WriteLine(errstate.ToString());
            if (errstate == WaypointMissionState.READY_TO_EXECUTE) {
                var startErr = await DJISDKManager.Instance.WaypointMissionManager.GetWaypointMissionHandler(0).StartMission();
                 System.Diagnostics.Debug.WriteLine(startErr.ToString());
            }
        }

        private async void Btn_AirraftName_Click(object sender, RoutedEventArgs e)
        {
            string str = null;
            var type = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).GetCameraTypeAsync();
            string str1 = type.value.Value.value.ToString();
            if(str1 != null)
            {
                str = "AirCraft Name is: " + type.value.Value.value.ToString();
            }


            try
            {
                var type2 = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(1, 0).GetCameraTypeAsync();
                string str2 = type2.value.Value.value.ToString();
                if (str2 != null)
                {
                    str = str + "AirCraft2 Name is: " + str2;
                }
            }
            catch
            {
                System.Diagnostics.Debug.WriteLine("No 2nd Drone");
            }


            //tblock_AircraftName.Text = str;

        }

        private async Task SendMessage(string message, string ip, int port)
        {
            using (var stream = await socket.GetOutputStreamAsync(new Windows.Networking.HostName(ip), port.ToString()))
            {
                using (var writer = new DataWriter(stream))
                {
                    var data = Encoding.UTF8.GetBytes(message);

                    writer.WriteBytes(data);
                    await writer.StoreAsync();
                }
            }
        }

        private async Task SendImgMessage(byte[] message, string ip, int port)
        {
            using (var stream = await socket.GetOutputStreamAsync(new Windows.Networking.HostName(ip), port.ToString()))
            {
                using (var writer = new DataWriter(stream))
                {
                    //var data = Encoding.UTF8.GetBytes(message);

                    writer.WriteBytes(message);
                    await writer.StoreAsync();
                }
            }
        }

        private async void OnLocationChanged(object sender, LocationCoordinate2D? value)
        {

            //var currentLocation = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAircraftLocationAsync();
            var currentLocation = value;
            _currentLocation = value;
            var currentAltitude = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAltitudeAsync();
            _currentAltitude = currentAltitude;
            try
            {
                //double nowLat = currentLocation.value.Value.latitude;
                //double nowLang = currentLocation.value.Value.longitude;
                double nowLat = currentLocation.Value.latitude;
                double nowLang = currentLocation.Value.longitude;
                double nowAltitude = currentAltitude.value.Value.value;

                string str = "lat:" + nowLat.ToString() + "lng:" + nowLang.ToString() + "alt:" + nowAltitude.ToString();
                //System.Diagnostics.Debug.WriteLine(str);

                //string adress = IPAddress.Broadcast.ToString();
                //await SendMessage(str, adress, port);
            } catch { }



            

            
        }
        

        private async void GetControllerState(object sender, object e)
        {

            double leftStickX = (await DJISDKManager.Instance.ComponentManager.GetRemoteControllerHandler(0, 0).GetRCStickLeftHorizontalAsync()).value.Value.value;
            double leftStickY = (await DJISDKManager.Instance.ComponentManager.GetRemoteControllerHandler(0, 0).GetRCStickLeftVerticalAsync()).value.Value.value;
            double rightStikX = (await DJISDKManager.Instance.ComponentManager.GetRemoteControllerHandler(0, 0).GetRCStickRightHorizontalAsync()).value.Value.value;
            double rightStikY = (await DJISDKManager.Instance.ComponentManager.GetRemoteControllerHandler(0, 0).GetRCStickRightVerticalAsync()).value.Value.value;
            //System.Diagnostics.Debug.WriteLine(rightStikX.ToString());

            var velocity = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetVelocityAsync();
            var velocityX = velocity.value.Value.x;
            var velocityY = velocity.value.Value.y;
            var velocityZ = velocity.value.Value.z;
            
            var currentLocation = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAircraftLocationAsync();
            var rotation = (await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAttitudeAsync()).value.Value.yaw;

            var modVelocityX = -1*velocityX*Math.Sin(rotation*Math.PI/180)+ velocityY * Math.Cos(rotation * Math.PI / 180);
            var modVelocityY = velocityY * Math.Sin(rotation * Math.PI / 180) + velocityX * Math.Cos(rotation * Math.PI / 180);
            //System.Diagnostics.Debug.WriteLine("modX:" + velocityX.ToString());
            //System.Diagnostics.Debug.WriteLine("modY:" + velocityY.ToString());
            var currentAltitude = await DJISDKManager.Instance.ComponentManager.GetFlightControllerHandler(0, 0).GetAltitudeAsync();
            try
            {
                //double nowLat = currentLocation.value.Value.latitude;
                //double nowLang = currentLocation.value.Value.longitude;
                double nowLat = currentLocation.value.Value.latitude;
                double nowLang = currentLocation.value.Value.longitude;
                double nowAltitude = currentAltitude.value.Value.value;
                
                string str = "lat:" + nowLat.ToString() + "lng:" + nowLang.ToString() + "alt:" + nowAltitude.ToString()+"lsX:" + leftStickX.ToString()+"lsY:"+leftStickY.ToString()+"rsX:"+rightStikX.ToString()+"rsY:"+rightStikY.ToString() + "vlX:" + modVelocityX.ToString() + "vlY:" + modVelocityY.ToString() + "vlZ:" + velocityZ.ToString() + "rot:" + rotation.ToString()+"gnd:"+_isGround.ToString()+"arm:"+_arMode.ToString();
                //System.Diagnostics.Debug.WriteLine(str);
                //string adress = "172.20.19.194";
                string adress = IPAddress.Broadcast.ToString();
                await SendMessage(str, adress, port);
                
                if (_isRecord)
                {
                    var dtime = System.DateTime.Now;
                    string crrrTime = dtime.Year.ToString() +"/"+ dtime.Month.ToString() +"/"+ dtime.Day.ToString() +" "+ dtime.Hour.ToString() +":"+ dtime.Minute.ToString() + ":"+dtime.Second.ToString()+"."+dtime.Millisecond.ToString();
                    //_writeData = _writeData +crrrTime+","+ nowLat.ToString() + "," + nowLang.ToString() + "," + nowAltitude.ToString() + "," + leftStickX.ToString() + "," + leftStickY.ToString() + "," + rightStikX.ToString() + "," + rightStikY.ToString() + "," + modVelocityX.ToString() + "," + modVelocityY.ToString() + "," + velocityZ.ToString() + "," + rotation.ToString() + "\r\n";
                    //await FileIO.WriteTextAsync(_stFile, _writeData);
                    _writeData = crrrTime + "," + nowLat.ToString() + "," + nowLang.ToString() + "," + nowAltitude.ToString() + "," + leftStickX.ToString() + "," + leftStickY.ToString() + "," + rightStikX.ToString() + "," + rightStikY.ToString() + "," + modVelocityX.ToString() + "," + modVelocityY.ToString() + "," + velocityZ.ToString() + "," + rotation.ToString() + "\r\n";
                    await FileIO.AppendTextAsync(_stFile, _writeData);
                }
                

                //DJISDKManager.Instance.VirtualRemoteController.UpdateJoystickValue((float)leftStickY * 1.5f, (float)leftStickX * 1.5f, (float)rightStikY * 1.5f, (float)rightStikX * 1.5f);
            }
            catch
            {
                System.Diagnostics.Debug.WriteLine("catched");
            }

            
        }


        protected override void OnNavigatedTo(NavigationEventArgs e)
        {
            _timer = new DispatcherTimer();

            _timer.Interval = TimeSpan.FromSeconds(0.05);
            _timer.Tick += GetControllerState;

            this.Loaded += delegate { this.Focus(FocusState.Programmatic); };
            //SystemNavigationManagerPreview.GetForCurrentView().CloseRequested += StopRecord;
            //_timer.Start();
        }

        private void Btn_StartWrite_Click(object sender, RoutedEventArgs e)
        {
            _isWitten = true;
        }

        

        private async void SwapChainPanel_PointerPressed(object sender, PointerRoutedEventArgs e)
        {
            
            var ptrE = e.Pointer;
         
            var ptrID = ptrE.PointerId;
            if (ptrID == 1)
            {
                var view = ApplicationView.GetForCurrentView();
                if (view.IsFullScreenMode)
                {
                    view.ExitFullScreenMode();
                    var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StopRecordAsync();
                    _isRecord = false;
                }
                else
                {
                    view.TryEnterFullScreenMode();
                    var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartRecordAsync();
                    _isRecord = true;
                }
            }
            else
            {
                //System.Diagnostics.Debug.WriteLine(ptrID);
                var tgt = e.GetCurrentPoint(swapChainPanel);
                System.Diagnostics.Debug.WriteLine(tgt.Position.X);
            }
            
        }

        private async void SetCameraWorkMode(CameraWorkMode mode)
        {
            if (DJISDKManager.Instance.ComponentManager != null)
            {
                CameraWorkModeMsg workMode = new CameraWorkModeMsg
                {
                    value = mode,
                };
                var retCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).SetCameraWorkModeAsync(workMode);
                if (retCode != SDKError.NO_ERROR)
                {
                    //OutputTB.Text = "Set camera work mode to " + mode.ToString() + "failed, result code is " + retCode.ToString();
                    System.Diagnostics.Debug.WriteLine("Set camera work mode to " + mode.ToString() + "failed, result code is " + retCode.ToString());
                }
                else
                {
                    //var recCode = await DJISDKManager.Instance.ComponentManager.GetCameraHandler(0, 0).StartRecordAsync();
                }
            }
            else
            {
                //OutputTB.Text = "SDK hasn't been activated yet.";
            }
        }

        private void C1Pushed(object sender, BoolMsg? value)
        {
            if (value.Value.value)
            {
                //Left Button
                _isGround++;
                _isGround = _isGround % 2;
                //System.Diagnostics.Debug.WriteLine(_isGround);
            }
            
        }

        private void C2Pushed(object sender, BoolMsg? value)
        {
            //Right Button
            if (value.Value.value)
            {
                _arMode++;
                _arMode = _arMode % 3;
                System.Diagnostics.Debug.WriteLine(_arMode);
            }

        }

    }

   
}
