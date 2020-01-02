from source.function import reward_function


def main():
    params = {
        'progress': 30,
        'track_width': 1.5,
        'heading': 0.0,
        'waypoints': [[1, 1], [2, 1], [3, 1], [4, 1], [5, 1], [5, 2], [5, 3], [5, 4], [5, 5], [4, 5], [3, 5], [2, 5],
                      [1, 5], [1, 4], [1, 3], [1, 2]],
        'closest_waypoints': [2, 3],
        'x': 3.5,
        'y': 1
    }
    score = reward_function(params)
    print("score: " + str(score))


if __name__ == "__main__":
    main()
