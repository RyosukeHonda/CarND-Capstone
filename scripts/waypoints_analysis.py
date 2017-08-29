"""
Very simple script for waypoints analysis
"""
import numpy as np
import matplotlib.pyplot as plt
import seaborn


def get_statistics(data):
    distances = []

    for first_element, second_element in zip(data[:-1], data[1:]):
        x_difference = second_element[0] - first_element[0]
        y_difference = second_element[1] - first_element[1]
        distance = np.sqrt(x_difference ** 2 + y_difference ** 2)
        distances.append(distance)

    statistics = "Min: {}, mean: {}, max: {}, std: {}".format(
        np.min(distances), np.mean(distances), np.max(distances), np.std(distances))

    return statistics


def main():

    path = "/tmp/waypoints/final_waypoints_6671.txt"
    data = np.loadtxt(path)

    plt.scatter(data[:, 0], data[:, 1], c="b")
    plt.show()

    print(get_statistics(data))


if __name__ == "__main__":
    main()
