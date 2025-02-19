import os
import pandas as pd

# Define the path to the CSV file in your Downloads folder
csv_file = os.path.expanduser("~/Downloads/test_data.csv")

# Read the CSV file
data = pd.read_csv(csv_file)

# Print the first few lines of the DataFrame
print(data.head())