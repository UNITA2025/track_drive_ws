# track_drive_ws

        self.centroid_sub = self.create_subscription(
          ConeInfoArray, '/cones/cone_info_right', self.centroid_callback, 10
        )
        self.centroid_sub = self.create_subscription(
          ConeInfoArray, '/cones/cone_info_left', self.centroid_callback, 10
        )
