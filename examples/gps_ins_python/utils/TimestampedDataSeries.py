from bisect import bisect_left, bisect_right


def find_range(array, a, b):
    start = bisect_right(array, a)
    end = bisect_left(array, b)
    return (start, end)


class TimestampedData:
    def __init__(self, timestamp, data):
        self.timestamp = timestamp
        self.data = data


class TimestampedDataSeries:
    def __init__(self):
        self.timestamps = []
        self.data = []

    def empty(self):
        return not self.timestamps

    def insert(self, key, value):
        if self.empty():
            self.timestamps.append(key)
            self.data.append(value)
        elif key < self.timestamps[-1]:
            self.timestamps.append(key)
            self.data.append(value)
            sorted_data = sorted(zip(self.timestamps, self.data))
            self.data = [data for _, data in sorted_data]
            self.time = [timestamps for timestamps, _ in sorted_data]
        else:
            self.timestamps.append(key)
            self.data.append(value)

    def insert_timestamped_data(self, timestamped_data):
        if isinstance(timestamped_data, TimestampedData):
            self.insert(timestamped_data.timestamp, timestamped_data)
        else:
            raise Exception(
                "Insert data as key, value pair or TimestampedData object"
            )

    def front(self):
        return self.data[0]

    def back(self):
        return self.data[-1]

    def get_in_range(self, t0, t1):
        '''
        Gets elements in the range [t0, t1].

        Parameters:
        t0: beginning of range
        t1: end of range

        Returns:
        List of the elements in the range
        '''
        start = bisect_right(self.timestamps, t0)
        end = bisect_left(self.timestamps, t1)
        if end != 0 and len(self.timestamps) > end:
            if self.timestamps[end] == t1:
                end = end + 1
        return self.data[start:end]

    def erase(self, items_to_erase):
        '''
        Erases elements in the range items_to_erase
        '''
        for item in items_to_erase:
            index = self.timestamps.index(item.timestamp)
            self.timestamps.pop(index)
            self.data.pop(index)
        return

    def pop_front(self):
        self.timestamps.pop(0)
        return self.data.pop(0)
