# Qualisys Bridge for ROS 2

This package bridges motion capture data from the Qualisys system into the ROS 2 ecosystem. It reads 6DOF body data from QTM and republishes it as ROS 2 odometry messages.

---

## 🔧 Configuration

The configuration file is located at:  
`config/bridge.yaml`

You must adapt the following parameters to match your setup:

### 1. `server_address`
Set this to the IP address of the computer running the Qualisys software (QTM).

### 2. `qtm_body_names`
A list of rigid body names as defined in Qualisys.  
Only bodies listed here will be processed.

### 3. `ros_body_names`
The corresponding names/namespaces to be used in ROS 2.

### 4. `latch_timeout`
Keep the value `false` by default.

### 5. `publish_visual_odometry`
Keep the value `true` by default.

> **Important:**  
> - All lists must be of the same length.  
> - The order of elements in each list must match across parameters.  
> - You can use as many bodies as needed.

**Example:**

```yaml
qtm_body_names: ['uvms', 'christmasStar']
ros_body_names: ['klopsi00', 'star']
server_address: '192.168.0.181'
latch_timeout: [false, false]
publish_visual_odometry: [true, true]
```

In the above setup the rigid body named uvms in Qualisys is exposed as /klopsi00/... in ROS 2.