import cv2
from datetime import datetime





class WebcamFeed:
    """
        A class to capture and display frames from a webcam.

        Attributes
        ----------
        camera_index : int
            The index of the camera to use. Defaults to 1.
        cap_show : int
            The flag to use when opening the camera. Defaults to cv2.CAP_DSHOW.
        window_name : str
            The name of the window to display the webcam feed in. Defaults to 'Webcam Feed'.
        cap : cv2.VideoCapture
            The VideoCapture object to capture frames from the webcam.
    """
    def __init__(self,
                 camera_index: int = 1,
                 cap_show: int = cv2.CAP_DSHOW,
                 window_name: str = 'Webcam Feed'):

        print("Starting webcam feed...")
        current_time = datetime.now().strftime("%H:%M:%S")

        self.camera_index = camera_index
        self.cap_show = cap_show
        self.window_name = window_name + ' - Press q to quit - Started at ' + str(current_time)
        self.cap = cv2.VideoCapture(self.camera_index, self.cap_show)

        if not self.cap.isOpened():
            raise ValueError("Error: Could not open webcam.")

        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        print("WebcamFeed initialized.")

    def capture_and_display_loop(self):
        """
        Continuously capture and display frames from the webcam.
        :return: None
        """
        while True:
            self.single_capture_and_display()

            if self.user_has_quit():
                break

        self.release_resources()

    def single_capture_and_display(self):
        """
        Capture and display a single frame from the webcam. Does not loop --> Can be called from within another loop.
        :return: The captured frame as an ndarray. If the frame capture is unsuccessful, the frame will be None.
        """
        ret, frame = self.cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            return None

        # cv2.imshow(self.window_name, frame)

        return frame

    def release_resources(self):
        """
        Release the webcam and close all windows.
        :return: None
        """
        self.cap.release()
        cv2.destroyAllWindows()

    def user_has_quit(self):
        """
        Check if the user has pressed the 'q' key.
        :return: True if the user has pressed 'q', False otherwise.
        """
        return cv2.waitKey(1) & 0xFF == ord('q')


if __name__ == "__main__":
    # Sample usage:
    webcam = WebcamFeed()

    while True:
        webcam.single_capture_and_display()
        if webcam.user_has_quit():
            webcam.release_resources()
            break

    print("Done.")
