"""
Simple script that analyzes model accuracy at traffic lights prediction for various levels of cropping.
Crops are centered on image center
"""

import os
import glob
import pprint

import keras
import cv2
import tqdm
import numpy as np

import utilities


def process_image(image):

    desired_shape = (128, 128)
    image = cv2.resize(image, desired_shape, cv2.INTER_LINEAR)
    image = image.astype('float32') / 255
    processed_image = image.reshape(1, *image.shape)

    return processed_image


def crop_image(image, margin):
    """
    Crops margin number of pixels on each side
    :param image: numpy arrray
    :param margin: int
    :return: numpy array
    """

    y_size = image.shape[0]
    x_size = image.shape[1]

    return image[margin:y_size - margin, margin:x_size - margin]


def crop_image_relative(image, relative_margin):
    """
    Crops margin number of pixels on each side by a margin relative to image size in that dimension
    :param image: numpy arrray
    :param relative_margin: float
    :return: numpy array
    """

    y_size = image.shape[0]
    y_margin = int(relative_margin * y_size)

    x_size = image.shape[1]
    x_margin = int(relative_margin * x_size)

    return image[y_margin:y_size - y_margin, x_margin:x_size - x_margin]


def get_predictions_accuracy(model, images, class_id, margin):

    # cropped_images = [crop_image(image, margin) for image in images]
    cropped_images = [crop_image_relative(image, margin) for image in images]
    processed_images = [process_image(image) for image in cropped_images]

    predicted_ids = []

    for image in processed_images:

        predicted_class_id = model.predict_classes(image, batch_size=1, verbose=0)
        predicted_ids.append(predicted_class_id[0])

    # cv2.imshow("image", cropped_images[0])
    # cv2.waitKey(0)

    accuracy = np.mean(np.array(predicted_ids) == class_id)
    return accuracy


def get_accuracy_report(accuracies):

    report = []

    for data in accuracies:

        entry = "{:04.2f}: {:05.3f}".format(data[0], data[1])
        report.append(entry)

    return report


def get_confusion_matrix(model, images_map, classes_ids):

    matrix = np.zeros(shape=(len(classes_ids), len(classes_ids)))

    for true_class_id, images in images_map.items():

        processed_images = [process_image(image) for image in images]

        for image in processed_images:

            predicted_class_id = model.predict_classes(image, batch_size=1, verbose=0)[0]
            matrix[true_class_id, predicted_class_id] += 1

    return matrix


def main():

    model_dir = "/home/student/CarND-Capstone/ros/src/tl_detector/light_classification/saved_models/"
    model_path = os.path.join(model_dir, "model128.json")
    weight_path = os.path.join(model_dir, "weights128.hdf5")

    model = utilities.get_model(model_path, weight_path)

    data_dir = "/home/student/data_partition/data/bag_dump_loop_with_traffic_light/"
    # data_dir = "/home/student/data_partition/data/bag_dump_just_traffic_light/"

    red_images = utilities.get_images_at_path(os.path.join(data_dir, "red"))
    yellow_images = utilities.get_images_at_path(os.path.join(data_dir, "yellow"))
    green_images = utilities.get_images_at_path(os.path.join(data_dir, "green"))
    other_images = utilities.get_images_at_path(os.path.join(data_dir, "nolight")) + \
                   utilities.get_images_at_path(os.path.join(data_dir, "unidentified"))

    images_map = {0: red_images, 1: yellow_images, 2: green_images, 3: other_images}
    classes_ids = [0, 1, 2, 3]

    matrix = get_confusion_matrix(model, images_map, classes_ids)
    print("Order: red, yellow, green, others")
    print(matrix)

    # green_accuracies = []
    # red_accuracies = []
    #
    # relative_margins = np.arange(0, 0.5, 0.05)
    #
    # for margin in tqdm.tqdm(relative_margins):
    #
    #     green_accuracy = get_predictions_accuracy(model, green_images, class_id=2, margin=margin)
    #     green_accuracies.append((margin, green_accuracy))
    #
    #     red_accuracy = get_predictions_accuracy(model, red_images, class_id=0, margin=margin)
    #     red_accuracies.append((margin, red_accuracy))
    #
    # green_accuracies = sorted(green_accuracies, key=lambda x: x[1], reverse=True)
    # red_accuracies = sorted(red_accuracies, key=lambda x: x[1], reverse=True)
    #
    # print("For data: {}".format(os.path.dirname(data_dir)))
    # print("Sorted green accuracies: {}".format(get_accuracy_report(green_accuracies)))
    # print("Sorted red accuracies: {}".format(get_accuracy_report(red_accuracies)))


if __name__ == "__main__":

    main()
