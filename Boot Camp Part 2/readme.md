# display_markers

Initializasion of transform listener   

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
Initializasion of transform broadcaster  

        self.tf_broadcaster = TransformBroadcaster(self)  
Subscribe to aruco marker topic    

        self.subscription = self.create_subscription(
                MarkerArray,
                '/aruco/markers',
                self.marker_callback,
                10
            )
Use lookup_transform function to get the transform from marker frame to map frame , and publish the transform through transform broadcaster.
          
        t = None
        for marker in msg.markers:

            try:
                t = self.tf_buffer.lookup_transform (
                    'map',                       # frame to transfrom into
                    f'aruco/marker{marker.id}',  # Input frame
                    rclpy.time.Time(),           # used to tell transfrom functions get the latest avilable tranform.
                    rclpy.duration.Duration(seconds=1.0) # wait up to 1 second for the TF to be available.
                    )
                
            except tf2_ros.LookupException as e:
                self.get_logger().warn(f"TF lookup error: {e}")  
                
  Initialize a instance of object TransformStamped() serves as transform msg to be broadcasted  
      
            tmsg = TransformStamped()
            tmsg.header.stamp = msg.header.stamp                 # keep msgs' timestamp
            tmsg.header.frame_id = 'map'                         # paerent frame in TF tree
            tmsg.child_frame_id  = f'aruco/detected{marker.id}'  # child frame in TF tree  
 
 Assign transformed translation and mannually-revised rotation to trasnform msg.  
       
            tmsg.transform.translation = t.transform.translation
            
            quaternion = (marker.pose.pose.orientation.x,
                          marker.pose.pose.orientation.y,
                          marker.pose.pose.orientation.z,
                          marker.pose.pose.orientation.w)

            if (marker.id == 2):
                rotation_to_apply = (m.pi, 0, m.pi)
            if (marker.id == 1):
                rotation_to_apply = (m.pi, 0, 0)  # first make sure how origin orientation can be retated into ground truth.
                
            quaternion_to_apply = quaternion_from_euler(*rotation_to_apply)  #convert the rotation angles from euler to quaternion representation.
            resulting_quaternion = quaternion_multiply(quaternion, quaternion_to_apply)  #finally multiply them to get rotated quaterion of objects.

Assign rotated quaterion to tf msg and broadcast it.  

            tmsg.transform.rotation.x = resulting_quaternion[0]
            tmsg.transform.rotation.y = resulting_quaternion[1]
            tmsg.transform.rotation.z = resulting_quaternion[2]
            tmsg.transform.rotation.w = resulting_quaternion[3]

            self.tf_broadcaster.sendTransform(tmsg)
        
