import csv

# Function to read data from the text file and convert it into a list of dictionaries
def read_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        entry = {}
        for line in lines:
            if "Current X position is" in line:
                entry["X"] = int(line.split(":")[-1].strip())
            elif "Current Y position is" in line:
                entry["Y"] = int(line.split(":")[-1].strip())
            elif "Current Z position is" in line:
                entry["Z"] = int(line.split(":")[-1].strip())
            elif "Current A position is" in line:
                entry["A"] = int(line.split(":")[-1].strip())
            elif "Current B position is" in line:
                entry["B"] = int(line.split(":")[-1].strip())
            elif "Current C position is" in line:
                entry["C"] = int(line.split(":")[-1].strip())
            elif line.strip() == "":
                # Empty line indicates the end of one set of data
                if entry:
                    data.append(entry)
                    entry = {}
        # Add the last entry if file does not end with a blank line
        if entry:
            data.append(entry)
    return data

# Function to get the B_Speed based on the index
def get_b_speed(idx):
    # Define the repeating pattern: 30, 30, 30, 44, 44, 44
    pattern = [63, 63, 63, 124, 124, 124]
    return pattern[idx % len(pattern)]

# Function to create a CSV file with the specified structure
def create_csv_from_text_file(input_filename, output_filename):
    data = read_data_from_file(input_filename)
    
    with open(output_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the header
        writer.writerow([
            "index", "X_position", "X_speed", "Y_position", "Y_speed", "Z_position", "Z_speed", 
            "A_position", "A_speed", "B_position", "B_speed", "C_position", "C_speed", 
            "image", "delay"
        ])
        
        # Write the rows based on the data
        index = 0  # Track the current row index
        for idx, entry in enumerate(data):
            # Original row
            row = [
                index,  # index
                entry["X"], 300,  # X_position, X_speed
                entry["Y"], 300,  # Y_position, Y_speed
                entry["Z"], 300,  # Z_position, Z_speed
                entry["A"], 100,  # A_position, A_speed
                entry["B"], get_b_speed(idx),   # B_position, B_speed (dynamic value based on index)
                entry["C"], 100,  # C_position, C_speed
                2,  # image (default value as per the example)
                2   # delay (default value as per the example)
            ]
            writer.writerow(row)
            index += 1
            
            # Additional row with Z_position + 2500
            row_plus_2500 = row.copy()
            row_plus_2500[0] = index  # update index
            row_plus_2500[5] = entry["Z"] + 2500  # Z_position increased by 2500
            writer.writerow(row_plus_2500)
            index += 1

            # Additional row with Z_position - 1500
            row_minus_1500 = row.copy()
            row_minus_1500[0] = index  # update index
            row_minus_1500[5] = entry["Z"] - 1500  # Z_position decreased by 1500
            writer.writerow(row_minus_1500)
            index += 1

            # Additional row with Z_position - 4000
            row_minus_4000 = row.copy()
            row_minus_4000[0] = index  # update index
            row_minus_4000[5] = entry["Z"] - 4000  # Z_position decreased by 4000
            writer.writerow(row_minus_4000)
            index += 1

# Run the function with the text file "table.txt" and output file "table.csv"
create_csv_from_text_file('table.txt', 'table.csv')
