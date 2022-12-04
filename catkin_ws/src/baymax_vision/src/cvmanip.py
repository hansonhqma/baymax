"""
computer vision image manipulation/viewing library for ease of access during development
runs on opencv
"""

import cv2 as cv
import numpy as np

def quickresize(frame, downscale):
    """
    fast image resize

    PARAMETERS

    frame : np.ndarray
        image array

    downscale : float
        image scale multiplier

    RETURNS

    np.ndarray
        resized image
    """
    dim = (int(frame.shape[1]/downscale), int(frame.shape[0]/downscale))
    return cv.resize(frame, dim)

def quickshow(img, window_name="frame"):
    """
    displays image array in opencv window

    PARAMETERS

    img : np.ndarray
        image array

    window_name : str
        window name
        defaults to "frame"

    RETURNS
    
    None
    """
    cv.imshow(window_name, img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        return

def quickview(source, downscale=None, label="frame", write=False):
    """
    displays video source in opencv window

    PARAMETERS

    downscale : float
        image scale multiplier
        defaults to None

    label : str
        window name
        defaults to "frame"

    RETURNS

    None
    """
    capture = cv.VideoCapture(source)
    
    if write:
        initial_frame = capture.read()[1]
        w = initial_frame.shape[1]
        h = initial_frame.shape[0]
        four_cc = cv.VideoWriter_fourcc(*'mp4v')
        writer = cv.VideoWriter("out.mp4", four_cc, 30, (w, h))
    
    frame_no = 0
    
    while True:
        ret, frame = capture.read()
        if not ret: break

        if downscale:
            frame = quickresize(frame, downscale)

        if write:
            writer.write(frame)
        
        cv.imshow(label, frame)
        frame_no += 1
        if cv.waitKey(1) & 0xFF == ord('s'):
            cv.imwrite(str(frame_no)+".jpg", frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            writer.release()
            return
