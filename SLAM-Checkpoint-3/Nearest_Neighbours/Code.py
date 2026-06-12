distances = np.array(distances)

        min_index = np.argmin(distances)
        min_distance = distances[min_index]

        if min_distance <= threshold:
            matches.append(min_index)
        else:
            matches.append(-1)

    return np.array(matches)
