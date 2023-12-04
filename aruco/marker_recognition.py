import cv2
import cv2.aruco as aruco
import numpy as np
import os

from opencv.webcam_input import WebcamFeed

MARKER_MIN_PERIMETER_RATE = 0.01
class ArUcoMarkerDetector:
    """
        A class to detect ArUco markers in a webcam feed.

        Attributes
        ----------
        webcam_feed : WebcamFeed
            The WebcamFeed object to capture frames from.
        marker_dict : cv2.aruco_Dictionary
            The dictionary of ArUco markers to use. Defaults to aruco.DICT_6X6_250.
        ----------
        """

    def __init__(self,
                 webcam: WebcamFeed,
                 thymio_marker_id = 0):

        self.webcam_feed = webcam
        self.thymio_marker_id = thymio_marker_id
        self.marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.transformation_matrix = None

    def detect_markers(self, base_frame=None):
        """
        Detect ArUco markers in the webcam feed. Display the frame with markers.
        :return: corners, ids, frame_markers, ids_to_direction -->
            The corners of the detected markers, the ids of the detected markers,
            the frame with markers drawn on it, the direction (orientation) of the markers.
        """
        if base_frame is None:
            base_frame = self.webcam_feed.single_capture_and_display()

        frame = base_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        parameters = aruco.DetectorParameters()
        parameters.minMarkerPerimeterRate = MARKER_MIN_PERIMETER_RATE
        # corners go clockwise from top left
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.marker_dict, parameters=parameters)

        ids_to_direction = {}

        if ids is not None:
            corners = np.array(corners)
            ids = np.array(ids)
            centroids = []
            for id in range(len(ids)):
                top_left = corners[id][0][0]
                bottom_left = corners[id][0][3]
                top_middle = 1 / 2 * (corners[id][0][0] + corners[id][0][1])

                # Calculate centroid (middle) of the marker
                centroid = np.mean(corners[id][0], axis=0)
                cv2.circle(frame, tuple(map(int, centroid)), 5, (0, 255, 0), -1)

                direction = top_middle - centroid
                ids_to_direction[id] = direction

                cv2.arrowedLine(frame, tuple(map(int, centroid)),
                                tuple(map(int, top_middle + direction)), (0, 255, 0), thickness=2, tipLength=0.1)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        if self.webcam_feed.user_has_quit():
            self.webcam_feed.release_resources()

        return corners, ids, frame_markers, ids_to_direction, base_frame

    def process_image_with_aruco_markers(self):
        """
        Process the image by adjusting it based on the detected ArUco markers.
        The image is transformed to place the markers 0-3 in the corners (top left then clockwise).
        :param frame: The frame to be processed.
        :return: The processed frame.
        """
        corners, ids, frame_markers, ids_to_direction, base_frame = self.detect_markers()

        if ids is None or len(ids) < 1:
            return frame_markers, corners, ids, frame_markers, ids_to_direction, base_frame

        detected_markers = zip(ids.reshape(-1), corners.reshape(-1, 4, 2))

        # Create a dictionary to store the detected markers' positions
        marker_for_corner = {0: None, 1: None, 2: None, 3: None}

        # Find the positions of the detected markers
        for marker_id, marker_corners in detected_markers:
            if (marker_id >= len(marker_for_corner) or marker_id < 0):
                continue
            marker_for_corner[marker_id] = marker_corners[marker_id]

        processed_frame = frame_markers
        h, w = frame_markers.shape[:2]

        # Don't recompute the transformation matrix if it has already been computed
        # as this leads to shaking of the image (pixle-wise adjustments to the image)
        if self.transformation_matrix is not None:
            base_frame, corners, processed_frame = self._transform_images(base_frame, corners, frame_markers, h,
                                                                          processed_frame, w)
            return processed_frame, corners, ids, frame_markers, ids_to_direction, base_frame

        if all(position is not None for position in marker_for_corner.values()):
            # Sort markers by id (transform need them sorted from top left clockwise 0-3)
            sorted_marker_positions = [marker_for_corner[i] for i in range(4)]
            new_corners = np.array(sorted_marker_positions)

            # Perspective transformation to adjust the image to have the markers in the corners
            destination_corners = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)

            self.transformation_matrix = cv2.getPerspectiveTransform(np.float32(new_corners), destination_corners)
            base_frame, corners, processed_frame = self._transform_images(base_frame, corners, frame_markers, h,
                                                                          processed_frame, w)

        return processed_frame, corners, ids, frame_markers, ids_to_direction, base_frame

    def _transform_images(self, base_frame, corners, frame_markers, h, processed_frame, w):
        processed_frame = cv2.warpPerspective(frame_markers, self.transformation_matrix, (w, h))
        base_frame = cv2.warpPerspective(base_frame, self.transformation_matrix, (w, h))
        corners = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), self.transformation_matrix).reshape(corners.shape)
        return base_frame, corners, processed_frame

    def get_image_corner_coordinates(self, corners, ids, image_corner_ids=[0, 1, 2, 3]):
        """
        Get the coordinates of the corners of an image.
        :param corners: The corners of the arUco markers.
        :param ids: The ids of the arUco markers in the image.
        :param image_corner_ids: The ids of the markers in the corners of the image.
        :return: The coordinates of the corners of the image.
        """
        image_corner_coordinates = []
        for i in range(len(ids)):
            if ids[i][0] in image_corner_ids:
                image_corner_coordinates.append(corners[i][0])

    def release_resources(self):
        self.webcam_feed.release_resources()


if __name__ == "__main__":
    # sample usage:
    webcam = WebcamFeed()
    aruco_detector = ArUcoMarkerDetector(webcam)

    while True:
        webcam.single_capture_and_display()

        if webcam.user_has_quit():
            webcam.release_resources()
            break

        processed_frame, corners, ids, frame_markers, ids_to_direction, base_frame = aruco_detector.process_image_with_aruco_markers()

        cv2.imshow("frame_markers", frame_markers)
        cv2.imshow("processed_frame", processed_frame)

    print("Done.")
