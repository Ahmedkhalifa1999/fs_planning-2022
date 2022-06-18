import numpy as np
from scipy.interpolate import CubicSpline

def find_interval(x_points,x):
    for idx,x_val in enumerate(x_points):
        if x_val > x:
            break
    return idx-1

class SplineInterpolator:
    def __init__(self):
        pass

    def fit(self,x_points,y_points):
        self.x_points = x_points
        self.spline = CubicSpline(x_points,y_points,bc_type="natural")
        self.coeffs = self.spline.c

    def evaluate(self,x):
        '''
        Parameters
        ----------
        x_points: list (or np.array) of floats
            Sorted ascendingly list of x values for points spline is fitted on
        '''
        # Improve by using binary search
        interval = find_interval(self.x_points,x)

        if interval < 0 or interval > self.coeffs.shape[1]:
            print("WARNING: Evaluating spline outside its bounds at value {}".format(x))
            
        if interval < 0: interval = 0
        
        ans = 0
        c = self.coeffs[:,interval]
        x -= self.x_points[interval]

        for idx,cv in enumerate(c[::-1]):
            ans += cv*(x**idx)
        
        return ans
    
class ArcLengthSpline:
    '''
    This class fits a parametric third order spline as a function of arclength
    Note: it also resamples the given points in order to have equal distances (euclidean) between each two knots
    Parameters
    ----------
        num_samples: int
            The number of knots create when resampling
        arclength_dt: float
            The delta time used when estimated the arclength
    Attributes
    ----------

    '''
    def __init__(self,num_samples=50,arclength_dt=0.01):
        self.arclength_dt = arclength_dt
        self.num_samples = num_samples

    def fit_spline(self,x_points,y_points, plot=False):
        # Create initial spline objects
        self.x_spline = SplineInterpolator()
        self.y_spline = SplineInterpolator()
        
        # Fit initial splines
        self.t_points = np.arange(0,x_points.shape[0])
        self.x_spline.fit(self.t_points,x_points)
        self.y_spline.fit(self.t_points,y_points)

        # Get arclengths of the whole spline
        arclengths,t_vals = self.compute_total_arclength()
        total_arclength = np.sum(arclengths)
        self.total_arclength = total_arclength

        self.delta_arclength = total_arclength/self.num_samples

        # Resample using self.delta_arclength between each two points
        new_t_vals = [0]  # Will have arclength from start to each point
        current_theta = self.delta_arclength
        new_x_points = [self.x_spline.evaluate(0)]
        new_y_points = [self.y_spline.evaluate(0)]

        for idx in range(arclengths.shape[0]):
            if arclengths[:idx].sum()>=current_theta or idx==arclengths.shape[0]-1:
                current_theta += self.delta_arclength
                new_t_val = t_vals[idx-1]
                new_t_vals.append(arclengths[:idx].sum())
                new_x_points.append(self.x_spline.evaluate(new_t_val))
                new_y_points.append(self.y_spline.evaluate(new_t_val))
                
        new_x_points = np.array(new_x_points)
        new_y_points = np.array(new_y_points)

        # Create new spline objects to fit resampled points
        self.x_spline = SplineInterpolator()
        self.y_spline = SplineInterpolator()
        
        # Fit using resampled points
        self.x_spline.fit(new_t_vals,new_x_points)
        self.y_spline.fit(new_t_vals,new_y_points)
        self.t_points = new_t_vals
        self.x_coeffs = self.x_spline.coeffs
        self.y_coeffs = self.y_spline.coeffs

        if plot:
            # Get resampled points from the fitted splines
            points = []
            for i in range(self.num_samples):
                theta = self.delta_arclength*i
                x,y = self.x_spline.evaluate(theta), self.y_spline.evaluate(theta)
                points.append([x,y])
            points = np.array(points)

            # Plot the points
            plt.title("Resampled points")
            plt.scatter(points[:,0],points[:,1])
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.show()

    def evaluate(self,theta):
        theta = np.clip(theta,0,self.total_arclength-.01)
        
        interval = int(theta//self.delta_arclength)
        if interval>= self.x_coeffs.shape[1]:
            print(self.total_arclength)
            print(self.delta_arclength)
            print(interval)
        ans_x, ans_y = 0,0
        cx = self.x_coeffs[:,interval][::-1]
        cy = self.y_coeffs[:,interval][::-1]
        d_theta = theta-interval*self.delta_arclength

        for idx in range(cx.shape[0]):
            poly_term = d_theta**idx
            ans_x += cx[idx]*poly_term
            ans_y += cy[idx]*poly_term
        
        return ans_x, ans_y

    def estimate_interval_arclength(self,interval):
        '''
        Estimates the arclength of one cubic parameteric curve from the splines

        Parameters
        ----------
        interval: int

        Returns
        -------
        lengths: np.array of floats
            Arclengths between points on the given parameteric curve that are self.arclength_dt apart
        t_vals: np.array of floats
            The values of t on points on the given parameteric curve that are self.arclength_dt apart
        '''
        x_coeffs = self.x_spline.coeffs[:,interval][::-1]
        y_coeffs = self.y_spline.coeffs[:,interval][::-1]

        t_vals = np.arange(interval,interval+1+self.arclength_dt,self.arclength_dt) - interval

        # Evalute the splines at points t_vals
        points_x = np.zeros(t_vals.shape[0])
        points_y = np.zeros(t_vals.shape[0])
        for idx in range(x_coeffs.shape[0]):
            points_x += x_coeffs[idx]*(t_vals**idx)
            points_y += y_coeffs[idx]*(t_vals**idx)

        # Calculate the distance between each 2 consecutive points
        x_diffs = points_x[:-1] - points_x[1:]
        y_diffs = points_y[:-1] - points_y[1:]
        x_diffs *= x_diffs
        y_diffs *= y_diffs
        dist_sums  = x_diffs + y_diffs
        lengths = np.sqrt(dist_sums)

        t_vals = t_vals[:-1] + interval

        return lengths, t_vals

    def compute_total_arclength(self):
        '''
        Estimate the arclength of the whole spline (all of the cubic polynomials fitted)

        Returns
        -------
        arclengths: np.array of floats
            Arclengths between points on the fitted spline that are self.arclength_dt apart
        t_vals: np.array of floats
            The values of t on points on the fitted spline that are self.arclength_dt apart
        '''
        arclengths = []
        t_vals = []
        for i in range(self.t_points.shape[0]-1):
            interval_arclengths,interval_t_vals = self.estimate_interval_arclength(i)

            arclengths.extend(interval_arclengths.tolist())
            t_vals.extend(interval_t_vals.tolist())
        
        arclengths = np.array(arclengths)
        t_vals = np.array(t_vals)

        return arclengths, t_vals
