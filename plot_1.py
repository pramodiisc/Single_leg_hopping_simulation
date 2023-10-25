import matplotlib.pyplot as plt
import pandas as pd
import scienceplots

# Read the CSV file
data = pd.read_csv('new_iter_power1.csv')

# Extract the two columns from the CSV file
x = data['kp_foot']
y = data['Average-power']

# Plot the data

plt.plot(x, y)

# Set the grid lines
plt.grid(True)

# Set the plot style to Science IEEE format
plt.style.use(['science','ieee'])

# Add labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Data Plot')

# Display the plot
plt.show()


# import matplotlib.pyplot as plt
# import pandas as pd

# # Read the CSV file
# data = pd.read_csv('new_iter_power1.csv')

# # Extract the two columns from the CSV file
# x = data['kp_foot']
# y = data['Average-power']

# # Plot the data
# plt.plot(x, y)

# # Set the grid lines
# plt.grid(True)

# # Set the plot style to 'ggplot'
# plt.style.use('ggplot')

# # Add labels and title
# plt.xlabel('X-axis')
# plt.ylabel('Y-axis')
# plt.title('Data Plot')

# # Display the plot
# plt.show()
# By setting plt.style.use('ggplot'), the script applies the 'ggplot' style to the plot. Feel free to explore other available styles in matplotlib by calling print(plt.style.available) to see a list of available styles and choose the one that suits your preferences. Additionally, you can manually customize various plot settings, such as fonts, colors, and line styles, to achieve the desired appearance.






