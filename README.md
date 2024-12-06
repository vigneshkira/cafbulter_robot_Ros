# **Butler Robot - ROS-Based Café Simulation**

## **Project Overview**

The **Butler Robot** project is a ROS-based message-passing system designed to simulate order processing and delivery in a café environment. The project involves message exchange between nodes that represent the **Customer**, **Butler**, and **Kitchen**. This system demonstrates how nodes can interact using the ROS 2 Publisher-Subscriber model for real-time communication.

The current implementation focuses on message passing between nodes, and future upgrades will include simulation in **Gazebo** and robot navigation using **path planning** algorithms.

## **Features**

1. **Message Passing Between Nodes**:
   - The **Customer Node** places orders by publishing messages.
   - The **Butler Node** processes and confirms orders.
   - The **Kitchen Node** simulates food preparation and notifies when the order is ready.

2. **ROS Communication**:
   - **Publishers**: Send orders, confirmations, and status updates.
   - **Subscribers**: Receive and process the corresponding messages.

3. **Simulation and Navigation** (Planned Features):
   - Integration with **Gazebo** for environment simulation.
   - Use of **Nav2** for path planning and robot navigation.

## **How to Run the Project**

### **Steps to Run the Package**
1. **Run the Customer Node**:
   The `test_node` handles user input and sends orders to the system. Use the following command to start it:
   ```bash
   ros2 run cafe_root test_node
   ```

2. **Run the Butler Launch File**:
   The Butler launch file manages the **Butler** and **Kitchen** nodes, which process orders and simulate food preparation. Start the launch file using:
   ```bash
   ros2 launch cafe_root butler_launch.py
   ```

### **Monitor ROS Topics**
- **View customer orders**:
  ```bash
  ros2 topic echo /customer/order
  ```
- **View confirmation status**:
  ```bash
  ros2 topic echo /confirmation_topic
  ```



## **Planned Upgrades**

1. **Simulation**:
   - Use **Gazebo** to create a virtual café environment.
   - Simulate the robot's movement between tables and the kitchen.

2. **Navigation**:
   - Implement **Nav2** for path planning.
   - Enable autonomous navigation for the robot to deliver orders.

3. **Enhanced Communication**:
   - Add error handling for dropped or delayed messages.
   - Introduce more detailed status updates for orders.

## **Future Directions**

This project will evolve from a message-passing system to a full simulation with navigation and interaction in a virtual environment. The addition of Gazebo and path planning will enhance its capability to demonstrate real-world robotics applications.

---

