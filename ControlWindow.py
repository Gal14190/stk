import cv2
import numpy as np

class ControlWindow:
    def __init__(self, vibers_motor, servos_motor, notification):
        # יצירת תמונה לבנה בגודל 400x800
        width, height = 800, 400
        image = np.ones((height, width, 3), dtype=np.uint8) * 255
        
        #התראות
        text_notif = f'Notification: {notification}'
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, text_notif, (10,20),
                    font, 1, (0,0,0), 2, cv2.LINE_AA)

        # ציור שלושה עיגולים אדומים
        circle_radius = 40
        circle_color = (0, 0, 255)  # BGR (אדום)
        circle_thickness = -1  # מילוי מלא

        circle_centers = [
            {'point': (width // 4, height // 4), 'color': (0,0,vibers_motor[0])},
            {'point': (width // 2, height // 4), 'color': (0,0,vibers_motor[1])},
            {'point': (3 * width // 4, height // 4), 'color': (0,0,vibers_motor[2])}
        ]

        for center in circle_centers:
            cv2.circle(image, center['point'], circle_radius, center['color'], circle_thickness)

        # ציור חץ ימינה
        arrow_color = (0, 0, 0)
        arrow_thickness = 3
        arrow_tip_length = 0.3

        # קואורדינטות לחץ ימינה
        start_point_right = (width // 2 + 50, height // 2 + 100)
        end_point_right = (start_point_right[0] + 150, start_point_right[1])

        cv2.arrowedLine(image, start_point_right, end_point_right, arrow_color, arrow_thickness, tipLength=arrow_tip_length)

        # הוספת הספרה 10 על החץ הימני
        text_right = str(servos_motor[1])
        cv2.putText(image, text_right, (start_point_right[0] + 50, start_point_right[1] - 10),
                    font, 1, arrow_color, 2, cv2.LINE_AA)

        # ציור חץ שמאלה
        start_point_left = (width // 2 - 50, height // 2 + 100)
        end_point_left = (start_point_left[0] - 150, start_point_left[1])

        cv2.arrowedLine(image, start_point_left, end_point_left, arrow_color, arrow_thickness, tipLength=arrow_tip_length)

        # הוספת הספרה 20 על החץ השמאלי
        text_left = str(servos_motor[0])
        cv2.putText(image, text_left, (end_point_left[0] + 50, end_point_left[1] - 10),
                    font, 1, arrow_color, 2, cv2.LINE_AA)

        # הצגת התמונה
        cv2.imshow("Control Window", image)
