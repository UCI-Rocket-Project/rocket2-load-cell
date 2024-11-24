import pandas as pd
import matplotlib.pyplot as plt

# Define the path to your CSV file
csv_file = r'C:\Users\david\Documents\rocket2-load-cell\code\loadcelltest\data.csv'  # Replace with the path to your actual CSV file

# Define column names for the data
column_names = ['Timestamp', 'Value']

# Read the CSV file
data = pd.read_csv(csv_file, header=None, names=column_names)

# Check the structure of the data
print(data.head())

# Extract columns for plotting
timestamps = data['Timestamp']
values = data['Value']

# Create the plot
plt.figure(figsize=(12, 6))
plt.plot(timestamps, values, label='Values over Time', marker='o', linestyle='-')

# Customize the plot
plt.title('Values vs. Timestamps', fontsize=16)
plt.xlabel('Timestamp', fontsize=14)
plt.ylabel('Value', fontsize=14)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)

# Adjust x-axis tick labels for better readability (optional)
plt.xticks(rotation=45)

# Show the plot
plt.tight_layout()
plt.show()
