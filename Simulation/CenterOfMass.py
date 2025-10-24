import numpy as np
import mujoco


class CenterOfMassCalculator:
    """
    Calculate and visualize the center of mass for the bipedal robot.
    """
    
    def __init__(self, model, data):
        """
        Initialize the center of mass calculator.
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        self.model = model
        self.data = data
        self.com_site_id = None
        self._setup_com_visualization()
    
    def _setup_com_visualization(self):
        """
        Set up visualization for center of mass in MuJoCo.
        This creates a visual site that will follow the COM.
        """
        # We'll add the visualization programmatically if it doesn't exist
        # For now, we'll just track the COM position
        pass
    
    def calculate_com(self):
        """
        Calculate the center of mass of the robot.
        
        Returns:
            numpy.ndarray: 3D position of the center of mass
        """
        # Calculate COM by summing weighted body positions (excluding world and floor)
        total_mass = 0.0
        com_pos = np.zeros(3)
        
        # Iterate through all bodies and calculate weighted COM
        for body_id in range(self.model.nbody):
            body_name = self.model.body(body_id).name
            
            # Skip world, floor, and any other non-robot bodies
            if body_id == 0 or body_name in ['world', 'Floor']:
                continue
            
            body_mass = self.model.body_mass[body_id]
            if body_mass > 0:
                body_pos = self.data.xpos[body_id]
                com_pos += body_mass * body_pos
                total_mass += body_mass
        
        if total_mass > 0:
            com_pos /= total_mass
        
        return com_pos
    
    def get_com_velocity(self):
        """
        Calculate the velocity of the center of mass.
        
        Returns:
            numpy.ndarray: 3D velocity of the center of mass
        """
        # Calculate COM velocity by summing weighted body velocities (excluding world and floor)
        total_mass = 0.0
        com_vel = np.zeros(3)
        
        for body_id in range(self.model.nbody):
            body_name = self.model.body(body_id).name
            
            # Skip world, floor, and any other non-robot bodies
            if body_id == 0 or body_name in ['world', 'Floor']:
                continue
            
            body_mass = self.model.body_mass[body_id]
            if body_mass > 0:
                # Get linear velocity from cvel (6D: angular + linear)
                body_vel = self.data.cvel[body_id][3:6]  # Linear velocity part
                com_vel += body_mass * body_vel
                total_mass += body_mass
        
        if total_mass > 0:
            com_vel /= total_mass
        
        return com_vel
    
    def get_support_polygon(self):
        """
        Calculate the support polygon based on foot contact points.
        
        Returns:
            list: Contact points defining the support polygon
        """
        contact_points = []
        
        # Check for contacts with the ground
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            
            # Get the body IDs for the geometries in contact
            try:
                body1_id = self.model.geom_bodyid[contact.geom1] if contact.geom1 < self.model.ngeom else -1
                body2_id = self.model.geom_bodyid[contact.geom2] if contact.geom2 < self.model.ngeom else -1
                
                body1_name = self.model.body(body1_id).name if body1_id >= 0 else ""
                body2_name = self.model.body(body2_id).name if body2_id >= 0 else ""
                
                # Check if contact involves a foot and the floor
                is_foot_contact = (("Foot" in body1_name or "Foot" in body2_name) and 
                                   ("Floor" in body1_name or "Floor" in body2_name))
                
                if is_foot_contact:
                    # This is a foot-floor contact
                    contact_points.append(contact.pos.copy())
                    
            except:
                continue
        
        return contact_points
    
    def is_statically_stable(self):
        """
        Check if the robot is statically stable by verifying if the COM projection
        falls within the support polygon.
        
        Returns:
            bool: True if statically stable, False otherwise
        """
        com_pos = self.calculate_com()
        contact_points = self.get_support_polygon()
        
        if len(contact_points) < 2:
            return False  # Need at least 2 contact points for stability
        
        # Project COM onto ground plane (z=0)
        com_projection = com_pos[:2]  # x, y coordinates only
        
        # For simplicity, check if COM is within a bounding box of contact points
        if len(contact_points) >= 2:
            contact_x = [point[0] for point in contact_points]
            contact_y = [point[1] for point in contact_points]
            
            min_x, max_x = min(contact_x), max(contact_x)
            min_y, max_y = min(contact_y), max(contact_y)
            
            return (min_x <= com_projection[0] <= max_x and 
                    min_y <= com_projection[1] <= max_y)
        
        return False
    
    def get_stability_margin(self):
        """
        Calculate the stability margin (distance from COM to support polygon edge).
        
        Returns:
            float: Stability margin (positive = stable, negative = unstable)
        """
        com_pos = self.calculate_com()
        contact_points = self.get_support_polygon()
        
        if len(contact_points) < 2:
            return -1.0  # Unstable
        
        # Simplified calculation: distance to nearest edge of support polygon
        # For a more accurate implementation, you'd use proper polygon algorithms
        com_projection = com_pos[:2]
        
        if len(contact_points) >= 2:
            contact_x = [point[0] for point in contact_points]
            contact_y = [point[1] for point in contact_points]
            
            min_x, max_x = min(contact_x), max(contact_x)
            min_y, max_y = min(contact_y), max(contact_y)
            
            # Distance to nearest edge
            dist_to_edges = [
                com_projection[0] - min_x,  # left edge
                max_x - com_projection[0],  # right edge
                com_projection[1] - min_y,  # front edge
                max_y - com_projection[1]   # back edge
            ]
            
            return min(dist_to_edges)
        
        return -1.0
    
    def get_com_info(self):
        """
        Get comprehensive center of mass information.
        
        Returns:
            dict: Dictionary containing COM position, velocity, stability info
        """
        com_pos = self.calculate_com()
        com_vel = self.get_com_velocity()
        is_stable = self.is_statically_stable()
        stability_margin = self.get_stability_margin()
        contact_points = self.get_support_polygon()
        
        return {
            'position': com_pos,
            'velocity': com_vel,
            'is_stable': is_stable,
            'stability_margin': stability_margin,
            'num_contacts': len(contact_points),
            'contact_points': contact_points
        }
    
    def visualize_com(self, viewer=None):
        """
        Visualize the center of mass in the MuJoCo viewer.
        This adds visual markers for COM and support polygon.
        
        Args:
            viewer: MuJoCo viewer instance (optional)
        """
        # This would be implemented to add visual markers in the viewer
        # For now, we'll return the information that can be displayed
        return self.get_com_info()


def add_com_visualization_to_model(model_xml_path, output_path=None):
    """
    Add center of mass visualization elements to the MuJoCo model XML.
    
    Args:
        model_xml_path: Path to the original model XML file
        output_path: Path to save the modified XML (optional)
    
    Returns:
        str: Modified XML content
    """
    with open(model_xml_path, 'r') as f:
        xml_content = f.read()
    
    # Add COM visualization site to the pelvis body
    com_site = '''
      <site name="com_marker" pos="0 0 0" size="0.03" rgba="1 0 0 0.8" type="sphere"/>'''
    
    # Insert the site into the Pelvis body
    pelvis_body_start = xml_content.find('<body name="Pelvis"')
    if pelvis_body_start != -1:
        # Find the end of the opening tag
        tag_end = xml_content.find('>', pelvis_body_start) + 1
        # Find the next line to insert the site
        next_line = xml_content.find('\n', tag_end) + 1
        
        # Insert the COM site
        xml_content = xml_content[:next_line] + com_site + xml_content[next_line:]
    
    if output_path:
        with open(output_path, 'w') as f:
            f.write(xml_content)
    
    return xml_content