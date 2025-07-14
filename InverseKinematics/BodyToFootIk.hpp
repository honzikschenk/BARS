/**
 * @brief Calculates the inverse kinematics to move a foot to a specific coordinate relative to the body.
 * @author Honzik Schenk
 * 
 * BodyToFootIK is a library that calculates the inverse kinematics necessary to move a foot to a specific
 * coordinate relative to the center of the body. The library goes off the assumption that there are two
 * legs and each have a 3-axis hip joint, a 1-axis knee joint, and a 1-axis ankle joint. The library also
 * assumes that the legs are symmetrical.
 */

#ifndef BODYTOFOOTIK_HPP
#define BODYTOFOOTIK_HPP

#include <vector>

class BodyToFootIK {
    public:
        BodyToFootIK();

        /**
         * @brief Calculates the inverse kinematics to move a foot to a specific coordinate relative to the body.
         * @param x
         * @param y
         * @param z
         * @return A vector containing the angles for all of the joints.
         */
        std::vector<double> calculateIK(double x, double y, double z);
    
    private:
        /**
         * @brief Converts radians to a range between min and max.
         * @param radians The angle in radians to convert.
         * @param positionMin The minimum position of the joint, typically equal to -pi/2.
         * @param positionMax The maximum position of the joint, typically equal to pi/2.
         * @param min The minimum value equal to -pi/2.
         * @param max The maximum value equal to pi/2.
         * @return The converted angle within the specified range.
         */
        double interpolateRange(double radians, double positionMin, double positionMax, double min, double max);
};

#endif // BODYTOFOOTIK_HPP
