from PeanutNumClassification import PeanutNumClassification
import cv2

PeanutNumClassifier = PeanutNumClassification()  # Initialize the classifier

img_Path = 'insufficient-sub.png'
img = cv2.imread(img_Path)  # read an image
class_name = PeanutNumClassifier.classify(img)  # Run classification

if class_name == 'sufficient':
    print("Peanut number is sufficient.")
elif class_name == 'insufficient':
    print("Peanut number is insufficient.")
elif class_name == 'operating':
    print("Operating...")