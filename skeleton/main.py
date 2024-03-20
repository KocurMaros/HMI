import cv2
import mediapipe as mp
import numpy as np
from udp_server import UDP_server

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
all_landmarks = np.zeros((75, 3), dtype=float)


def fill_lefthand(landmarks):
    for r in range(0, 21):
        all_landmarks[r][0] = landmarks.landmark[r].x
        all_landmarks[r][1] = landmarks.landmark[r].y
        all_landmarks[r][2] = landmarks.landmark[r].z


def fill_righthand(landmarks):
    for r in range(0, 21):
        all_landmarks[r + 21][0] = landmarks.landmark[r].x
        all_landmarks[r + 21][1] = landmarks.landmark[r].y
        all_landmarks[r + 21][2] = landmarks.landmark[r].z


def fill_pose(landmarks):
    for pose_point, r in zip(landmarks.landmark, range(0, 33)):
        if pose_point.visibility > 0.7:
            all_landmarks[r + 42][0] = pose_point.x
            all_landmarks[r + 42][1] = pose_point.y
            all_landmarks[r + 42][2] = pose_point.z


if __name__ == '__main__':
    # For webcam input:
    cap = cv2.VideoCapture(0)  # 'protocol://IP:port/1'
    UDPserver = UDP_server()
    with mp_pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as pose, \
            mp_hands.Hands(
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as hands:
        while cap.isOpened():
            all_landmarks.fill(0.0)
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                # If loading a video, use 'break' instead of 'continue'.
                continue

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = pose.process(image)
            results_h = hands.process(image)

            # Draw the pose annotation on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(
                image,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            if results.pose_landmarks:
                fill_pose(results.pose_landmarks)

            if results_h.multi_hand_landmarks:

                for hand_landmarks, hand_sides in zip(results_h.multi_hand_landmarks, results_h.multi_handedness):
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                    if hand_sides.classification[0].label == "Right":
                        fill_lefthand(hand_landmarks)
                    else:
                        fill_righthand(hand_landmarks)

            UDPserver.set_data(all_landmarks.tobytes())
            UDPserver.send_message()
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('Body + hands', cv2.flip(image, 1))
            if cv2.waitKey(5) & 0xFF == 27:
                break
    cap.release()
