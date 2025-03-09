ROS FLIR OpenCV camera driver for Boson and Lepton
========================

A driver to interface FLIR Lepton and Boson thermal cameras using v4l over usb.

cv_camera_node
------------------

This node uses [camera_info_manager](http://wiki.ros.org/camera_info_manager) for dealing with camera_info.
If no calibration data is set, it has dummy values except for width and height.

### Installation and Setup

For first-time set up of udev rules for the lepton or boson, run the following and unplug/replug the camera:

```
sudo sh -c "echo 'SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"1e4e\", ATTRS{idProduct}==\"0100\", SYMLINK+=\"pt1\", GROUP=\"usb\", MODE=\"666\"' > /etc/udev/rules.d/99-pt1.rules"
```

Often it's preferrable to use the serial number of a camera to ensure consistent ordering & calibration. To set up the driver to use SN#, plug each camera in in the order required, (ensuring no other USB cameras are plugged in with 'ls /dev/video*') and run the following command: 

```
udevadm info --name=/dev/video1 | grep 'E: ID_SERIAL_SHORT='
```

Then copy the whole SN# into the launch file param "~device_sn". If you do not see a number, check your udev rules as well as permissions on the device. Note that Leptons register two devices each, use the first (0, 2, 4... default behavior)

### Publish

* `~image_raw` (*sensor_msgs/Image*) - Mono16 radiometric raw data
* `~image_raw_viz` (*sensor_msgs/Image*) - colorized for visualization
* `~image_raw_cal` (*sensor_msgs/Image*) - normalized and possibly inverted for spatial calibration
* `~camera_info` (*sensor_msgs/CameraInfo*)

### Service

* `~set_camera_info` (*sensor_msgs/SetCameraInfo*)

### Parameters

* `~device_sn` (*string*, default: "") – serial number of IR camera to avoid id mismatches; see setup note.
* `~device_id` (*int*, default: 0) – capture device id, fallback if serial number is not specified.
* `~frame_id` (*string*, default: "camera") – `frame_id` of message header.
* `~image_width` (*int*) – try to set capture image width.
* `~image_height` (*int*) – try to set capture image height.
* `~camera_info_url` (*string*) – url of camera info yaml.
* `~file` (*string*, default: "") – if not "" then use movie file instead of device.
* `~capture_delay` (*double*, default: 0) – estimated duration of capturing and receiving the image.
* `~rescale_camera_info` (*bool*, default: false) – rescale camera calibration info automatically.

Supports CV_CAP_PROP_*, by below params; however, most are irrelevant to the FLIR Lepton/Boson. See example launch files.

* `~cv_cap_prop_pos_msec` (*double*)
* `~cv_cap_prop_pos_avi_ratio` (*double*)
* `~cv_cap_prop_frame_width` (*double*)
* `~cv_cap_prop_frame_height` (*double*)
* `~cv_cap_prop_fps` (*double*)
* `~cv_cap_prop_frame_count` (*double*)
* `~cv_cap_prop_format` (*double*)
* `~cv_cap_prop_mode` (*double*)
* `~cv_cap_prop_brightness` (*double*)
* `~cv_cap_prop_contrast` (*double*)
* `~cv_cap_prop_saturation` (*double*)
* `~cv_cap_prop_hue` (*double*)
* `~cv_cap_prop_gain` (*double*)
* `~cv_cap_prop_exposure` (*double*)
* `~cv_cap_prop_convert_rgb` (*double*)
* `~cv_cap_prop_rectification` (*double*)
* `~cv_cap_prop_iso_speed` (*double*)

And supports any props. Thanks to Hernan Badino!

* `~property_$(i)_code` (*int*) – set this code property using `~property_$(i)_value`, $(i) must start from 0.
* `~property_$(i)_value` (*double*) – the value to be set to `~property_$(i)_code`

If you want to set the property which code is 404 as 1,

```bash
rosrun cv_camera cv_camera_node _property_0_code:=404 _property_0_code:=1
```

If you want to set more, use `~property_1_code` and `~property_1_code`.

Nodelet
-------------------

This node works as nodelet (`cv_camera/CvCameraNodelet`).

Contributors
--------------------

This builds on the OTL/cv_camera and Kapernikov/cv_camera drivers, with customization for thermal camera peculiarities.

PR is welcome. I'll review your code to keep consistency, be patient.

FLIR adaptation:
* James Haley
Original cv_camera repo authors: 
* Oleg Kalachev
* Mikael Arguedas
* Maurice Meedendorp
* Max Schettler
* Lukas Bulwahn
* Kapernikov
