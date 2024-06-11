import pandas as pd
import sys
import re

def extract_id(info):
    # Extract the CAN ID from the Info column using regex
    match = re.search(r"ID: (\d+)", info)
    if match:
        return int(match.group(1))
    return None

def main(file_path, emitter_id):
    # Convert the emitter_id to the corresponding CAN IDs
    can_id_emitter_140 = 0x140 + emitter_id
    can_id_receiver_240 = 0x240 + emitter_id

    # Load the CSV file
    df = pd.read_csv(file_path)

    # Extract CAN IDs from the Info column
    df['ID'] = df['Info'].apply(extract_id)

    # Filter rows for the emitter and receiver CAN IDs
    df_140 = df[df['ID'] == can_id_emitter_140]
    df_240 = df[df['ID'] == can_id_receiver_240]

    # Display the filtered DataFrames for debugging
    print("Messages (ID: {}):".format(hex(can_id_emitter_140)))
    print(df_140)
    print("Messages (ID: {}):".format(hex(can_id_receiver_240)))
    print(df_240)

    # Calculate time differences between each 0x14* message and the closest 0x24* message (esclave)
    time_differences_14x_24x = []
    for time_14x in df_140['Time']:
        time_24x = df_240['Time'][df_240['Time'] > time_14x]
        if not time_24x.empty:
            time_differences_14x_24x.append((time_24x.iloc[0] - time_14x) * 1000)  # Convert to milliseconds

    # Calculate the average of the time differences (esclave)
    if time_differences_14x_24x:
        average_time_difference_14x_24x = sum(time_differences_14x_24x) / len(time_differences_14x_24x)
    else:
        average_time_difference_14x_24x = None

    print(f"Average time difference between CAN ID {hex(can_id_emitter_140)} and {hex(can_id_receiver_240)} (ms):", average_time_difference_14x_24x)

    # Calculate time differences between each 0x14* message and the next 0x14* message (maître)
    time_differences_14x_14x = []
    times_140 = df_140['Time'].values
    for i in range(len(times_140) - 1):
        time_differences_14x_14x.append((times_140[i + 1] - times_140[i]) * 1000)  # Convert to milliseconds

    # Calculate the average of the time differences (maître)
    if time_differences_14x_14x:
        average_time_difference_14x_14x = sum(time_differences_14x_14x) / len(time_differences_14x_14x)
    else:
        average_time_difference_14x_14x = None

    print(f"Average time difference between successive CAN ID {hex(can_id_emitter_140)} messages (ms):", average_time_difference_14x_14x)

    # Calculate the processing time of the maître
    if average_time_difference_14x_24x is not None and average_time_difference_14x_14x is not None:
        processing_time_maitre = average_time_difference_14x_14x - average_time_difference_14x_24x
    else:
        processing_time_maitre = None

    print(f"Processing time of the maître (ms):", processing_time_maitre)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python analyse_can.py <path_to_csv_file> <emitter_id>")
        sys.exit(1)

    file_path = sys.argv[1]
    emitter_id = int(sys.argv[2])  # Convert the emitter ID from string to integer
    main(file_path, emitter_id)

