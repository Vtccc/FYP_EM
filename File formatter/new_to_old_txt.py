# Define input and output file names
input_file = "//Users/tobycheung/Documents/VSCode/ISDN4001_4002/FYP_EM/File formatter/sailing_data009.txt"  # Replace with your actual new format file name
output_file = input_file.replace(".txt", "_compatible.txt")

# Define default values for missing fields
default_values = {
    "gps_satellites": 0,
    "gps_hdop": 0.0,
    "gps_age": 0,
    "gps_lat": 0.0,
    "gps_lng": 0.0,
    "gps_speed": 0.0,
    "gps_course": 0.0,
    "gps_month": 0,
    "gps_day": 0,
    "gps_year": 0,
    "gps_hour": 0,
    "gps_minute": 0,
    "gps_second": 0,
    "gps_centisecond": 0,
    "compass_azimuth": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "PM_out_val": 0,
    "runFastLED_task": 0,
}

# Process the file
try:
    with open(input_file, "r") as infile, open(output_file, "w") as outfile:
        for line in infile:
            fields = line.strip().split(",")

            # Extract GPS-related data from the new format
            gps_lat = float(fields[6]) if len(fields) > 6 else default_values["gps_lat"]
            gps_lng = float(fields[7]) if len(fields) > 7 else default_values["gps_lng"]
            gps_speed = float(fields[8]) if len(fields) > 8 else default_values["gps_speed"]
            gps_month = int(fields[9]) if len(fields) > 9 else default_values["gps_month"]
            gps_day = int(fields[10]) if len(fields) > 10 else default_values["gps_day"]
            gps_year = int(fields[11]) if len(fields) > 11 else default_values["gps_year"]
            gps_hour = int(fields[12]) if len(fields) > 12 else default_values["gps_hour"]
            gps_minute = int(fields[13]) if len(fields) > 13 else default_values["gps_minute"]
            gps_second = int(fields[14]) if len(fields) > 14 else default_values["gps_second"]
            gps_centisecond = int(fields[15]) if len(fields) > 15 else default_values["gps_centisecond"]

            # Construct old format row with only GPS data, leave other parameters as 0
            old_format_row = [
                default_values["gps_satellites"],  # gps.satellites.value()
                default_values["gps_hdop"],        # gps.hdop.hdop()
                default_values["gps_age"],         # gps.location.age()
                gps_lat,                           # gps.location.lat()
                gps_lng,                           # gps.location.lng()
                gps_speed,                         # gps.speed.mps()
                default_values["gps_course"],      # gps.course.deg()
                gps_month,                         # gps.date.month()
                gps_day,                           # gps.date.day()
                gps_year,                          # gps.date.year()
                gps_hour,                          # gps.time.hour()
                gps_minute,                        # gps.time.minute()
                gps_second,                        # gps.time.second()
                gps_centisecond,                   # gps.time.centisecond()
                default_values["compass_azimuth"], # compass_azimuth
                default_values["roll"],            # roll
                default_values["pitch"],           # pitch
                default_values["yaw"],             # yaw
                default_values["PM_out_val"],      # PM_out_val
                default_values["runFastLED_task"], # runFastLED_task
            ]

            # Write the converted row to the output file
            outfile.write(",".join(map(str, old_format_row)) + "\n")

    print(f"Formatted data has been written to {output_file}")

except FileNotFoundError:
    print(f"File {input_file} not found. Please ensure the file exists.")
except Exception as e:
    print(f"An error occurred: {e}")
