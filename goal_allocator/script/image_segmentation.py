import cv2
import numpy as np
import matplotlib.pyplot as plt


class edgeDetector():

    @staticmethod
    def get_equally_spaced_points(points, num_points):
        indices = np.linspace(0, len(points) - 1, num_points).astype(int)
        return points[indices]
    
    @staticmethod
    def process_image(image_path : str):
        # Read the image
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        flipped_img = cv2.flip(image,0)

        # Detect edges using Canny edge detector
        edges = cv2.Canny(flipped_img, 100, 200)
        
        # Get coordinates of the edge points
        edge_points = np.column_stack(np.where(edges > 0))

        # Function to select equally spaced points
        # Number of points you want to sample
        num_points = 4
        equally_spaced_points = edgeDetector.get_equally_spaced_points(edge_points, num_points)
        equally_spaced_swapped = []
        print(equally_spaced_points)
        for point in equally_spaced_points:
            cv2.circle(edges, tuple(point[::-1]), 2, (255, 0, 0), 2)  # Draw a red circle at each point
            equally_spaced_swapped.append(point[::-1])
        edge_flipped = cv2.flip(edges,0)
        return edge_flipped,equally_spaced_swapped
    





