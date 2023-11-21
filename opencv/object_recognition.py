import cv2
import numpy as np

from opencv.webcam_input import WebcamFeed
from aruco.marker_recognition import ArUcoMarkerDetector

THRESHOLD_MAX_VAL = 255
OBJECT_THRESHOLD = 40


class ObjectDetector:
    def __init__(self, aruco_marker_detector):
        self.aruco_marker_detector = aruco_marker_detector

    def detect_objects(self):
        processed_frame, corners, ids, frame_markers, ids_to_direction, base_frame = (
            self.aruco_marker_detector.process_image_with_aruco_markers())

        cv2.imshow('base Frame', base_frame)

        # cv2.imshow('Base Frame', base_frame)

        if (ids is not None and 0 in ids and 1 in ids and 2 in ids and 3 in ids):
            top_left_aruco = corners[np.where(ids == 0)[0][0]]
            top_right_aruco = corners[np.where(ids == 1)[0][0]]
            bottom_right_aruco = corners[np.where(ids == 2)[0][0]]
            bottom_left_aruco = corners[np.where(ids == 3)[0][0]]

            # Get the binary image with black objects on a white background, excluding ArUco markers
            binary_image = self._extract_black_objects(base_frame, [top_left_aruco, top_right_aruco, bottom_right_aruco,
                                                                   bottom_left_aruco])
        else:
            binary_image = self._extract_black_objects(base_frame, [])

        # Find contours of black objects
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the marker frame
        frame_with_objects = processed_frame.copy()
        cv2.drawContours(frame_with_objects, contours, -1, (0, 0, 255), 2)

        # Press 'q' to exit the loop
        if self.aruco_marker_detector.webcam_feed.user_has_quit():
            self.aruco_marker_detector.release_resources()


        return contours, binary_image, frame_with_objects, corners, ids

    def _extract_black_objects(self, frame, corner_aruco_markers):
        """
        Extract the black objects from a frame.
        :param frame: The frame to extract the black objects from.
        :param corner_aruco_markers: Corners of ArUco markers to be excluded.
        :return: The binary image with black objects on a white background.
        """

        # Convert corner_aruco_markers to numpy.int32
        corner_aruco_markers = np.array(corner_aruco_markers, dtype=np.int32)

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


        # Threshold the image to get binary image with black objects on white background
        # binary image is 0 where no black object is detected, 255 where black object is detected
        _, binary_image = cv2.threshold(gray_frame, OBJECT_THRESHOLD, THRESHOLD_MAX_VAL, cv2.THRESH_BINARY_INV)

        return binary_image

    # def _extract_black_objects(self, image, corner_aruco_markers):
    #     """
    #     Extract black objects on a white surface, excluding ArUco markers.
    #     :param image: The input image.
    #     :param corner_aruco_markers: The corners of the ArUco markers.
    #     :return: The image with black objects on a white surface.
    #     """
    #     # Create a mask to keep only the black objects
    #     mask = np.ones_like(image, dtype=np.uint8) * 255  # White background
    #
    #     for i in range(len(corner_aruco_markers)):
    #         # Exclude ArUco markers from the mask
    #         cv2.fillPoly(mask, [np.array(corner_aruco_markers[i], dtype=np.int32)], (0, 0, 0))
    #
    #     # Apply the mask to the original image
    #     result = cv2.bitwise_and(image, mask)
    #
    #     return result

    # def _extract_black_objects(self, image, corner_aruco_markers):
    #     """
    #     Extract black objects on a white surface, excluding ArUco markers.
    #     :param image: The input image.
    #     :param corner_aruco_markers: The corners of the ArUco markers.
    #     :return: The image with black objects on a white surface.
    #     """
    #     # Create a mask to keep only the black objects
    #     mask = np.ones_like(image, dtype=np.uint8) * 255  # White background
    #
    #     for i in range(len(corner_aruco_markers)):
    #         # Exclude ArUco markers from the mask
    #         print("got it")
    #         cv2.fillPoly(mask, [np.array(corner_aruco_markers[i], dtype=np.int32)], (255, 0, 0))
    #
    #     # Apply the mask to the original image
    #     result = image#cv2.bitwise_and(image, mask)
    #
    #     # Convert the result to a single-channel (grayscale) image
    #     gray_frame = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    #
    #
    #     # Threshold the image to get binary image with black objects on white background
    #     _, binary_image = cv2.threshold(gray_frame, 60, 255, cv2.THRESH_BINARY_INV)
    #
    #     # Exclude the ArUco markers from the binary image
    #     binary_image = cv2.bitwise_and(binary_image, cv2.bitwise_not(mask))
    #
    #     return binary_image



if __name__ == "__main__":
    # Sample usage:
    webcam = WebcamFeed()
    aruco_detector = ArUcoMarkerDetector(webcam)
    object_detector = ObjectDetector(aruco_detector)

    while True:
        contours, binary_image, frame_with_objects, corners, ids = object_detector.detect_objects()

        cv2.imshow('Object Detection', frame_with_objects)

        if webcam.user_has_quit():
            webcam.release_resources()
            break

    print("Done.")
