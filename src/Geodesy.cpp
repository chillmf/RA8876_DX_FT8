#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "arm_math.h"

#include "Geodesy.h"

#define WGS84_A 6378137.0
#define WGS84_F (1.0 / 298.257223563)
#define WGS84_B (WGS84_A * (1.0 - WGS84_F))
#define K_FACTOR (4.0 / 3.0) // standard atmospheric refraction

#define deg2rad(d) ((d) * M_PI / 180.0)
#define rad2deg(r) ((r) * 180.0 / M_PI)

// Vincenty's inverse solution
VincentyResult VincentyDistance(double lat1, double long1, double lat2, double long2)
{
    // Validate latitude and longitude ranges
    if (lat1 < -90.0 || lat1 > 90.0 || lat2 < -90.0 || lat2 > 90.0 ||
        long1 < -180.0 || long1 > 180.0 || long2 < -180.0 || long2 > 180.0)
    {
        return {0, NAN, NAN, false};
    }

    const double tolerance = 1e-12;
    const double a = WGS84_A;
    const double b = WGS84_B;
    const double f = WGS84_F;

    double deltaLongRad = deg2rad(long2 - long1);
    double reducedLat1 = atan((1 - f) * tan(deg2rad(lat1)));
    double reducedLat2 = atan((1 - f) * tan(deg2rad(lat2)));

    double sinReducedLat1 = sin(reducedLat1), cosReducedLat1 = cos(reducedLat1);
    double sinReducedLat2 = sin(reducedLat2), cosReducedLat2 = cos(reducedLat2);

    int iterLimit = 100;
    double lambda = deltaLongRad, lambdaP;
    double sinLambda, cosLambda, sinSigma, cosSigma, sigma;
    double sinAlpha, cosSqAlpha, cos2SigmaM, lambdaCorrection;

    do
    {
        sinLambda = sin(lambda);
        cosLambda = cos(lambda);
        sinSigma = sqrt((cosReducedLat2 * sinLambda) * (cosReducedLat2 * sinLambda) +
                        (cosReducedLat1 * sinReducedLat2 - sinReducedLat1 * cosReducedLat2 * cosLambda) *
                            (cosReducedLat1 * sinReducedLat2 - sinReducedLat1 * cosReducedLat2 * cosLambda));

        if (sinSigma == 0 || fabs(sinSigma) < tolerance)
            return {0, NAN, NAN, true}; // co-incident points

        cosSigma = sinReducedLat1 * sinReducedLat2 + cosReducedLat1 * cosReducedLat2 * cosLambda;
        sigma = atan2(sinSigma, cosSigma);
        sinAlpha = cosReducedLat1 * cosReducedLat2 * sinLambda / sinSigma;
        cosSqAlpha = 1 - sinAlpha * sinAlpha;

        if (cosSqAlpha == 0)
            cos2SigmaM = 0; // equatorial line
        else
            cos2SigmaM = cosSigma - 2 * sinReducedLat1 * sinReducedLat2 / cosSqAlpha;

        lambdaCorrection = f / 16 * cosSqAlpha * (4 + f * (4 - 3 * cosSqAlpha));
        lambdaP = lambda;
        lambda = deltaLongRad + (1 - lambdaCorrection) * f * sinAlpha *
                                    (sigma + lambdaCorrection * sinSigma *
                                                 (cos2SigmaM + lambdaCorrection * cosSigma *
                                                                   (-1 + 2 * cos2SigmaM * cos2SigmaM)));
    } while (fabs(lambda - lambdaP) > tolerance && --iterLimit > 0);

    if (iterLimit == 0)
    {
        // did not converge
        return {0, NAN, NAN, false};
    }

    double uSq = cosSqAlpha * (a * a - b * b) / (b * b);
    double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
    double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));
    double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) - B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));

    double fwdAz = atan2(cosReducedLat2 * sinLambda,
                         cosReducedLat1 * sinReducedLat2 - sinReducedLat1 * cosReducedLat2 * cosLambda);
    double revAz = atan2(cosReducedLat1 * sinLambda,
                         -sinReducedLat1 * cosReducedLat2 + cosReducedLat1 * sinReducedLat2 * cosLambda);

    return {b * A * (sigma - deltaSigma), fmod(rad2deg(fwdAz) + 360.0, 360.0), fmod(rad2deg(revAz) + 360.0, 360.0), true};
}

// LOS horizon distance with refraction (effective Earth radius method)
double RadioHorizon(double height)
{
    const double Reff = WGS84_A * K_FACTOR;
    return sqrt(2 * height * Reff);
}

// Validate characters for field (A-R), square (0-9), subsquare (a-x)
bool IsValidLocator(const char *locator)
{
    /*
    if (!locator)
    {
        return false;
    }

    size_t len = strlen(locator);
    if (len != 4 && len != 6)
    {
        return false;
    }

    // Field letters A–R (0–17)
    if (!isalpha(locator[0]) || toupper(locator[0]) < 'A' || toupper(locator[0]) > 'R')
        return false;
    if (!isalpha(locator[1]) || toupper(locator[1]) < 'A' || toupper(locator[1]) > 'R')
        return false;

    // Square digits 0–9
    if (!isdigit(locator[2]) || !isdigit(locator[3]))
        return false;

    if (len == 6)
    {
        // Subsquare letters a–x
        if (!isalpha(locator[4]) || tolower(locator[4]) < 'a' || tolower(locator[4]) > 'x')
            return false;
        if (!isalpha(locator[5]) || tolower(locator[5]) < 'a' || tolower(locator[5]) > 'x')
            return false;
    }

    return 1;

    */

    uint8_t A1, A2, N1, N2;
    uint8_t test = 0;

    A1 = locator[0] - 65;
    A2 = locator[1] - 65;
    N1 = locator[2] - 48;
    N2 = locator[3] - 48;

        if (A1 <= 17) // 'R'
            test++;
        if (A2 > 0 && A2 < 17)
            test++; // block RR73 Arctic and Antarctica
        if (N1 <= 9)
            test++;
        if (N2 <= 9)
            test++;

        if (test == 4)
            return true;
        else
            return false;

}

LatLong QRAtoLatLong(const char *locator)
{
    if (!IsValidLocator(locator))
    {
        return {0, 0, false};
    }

    char loc[7] = {0};
    strncpy(loc, locator, 6);
    for (int i = 0; loc[i]; i++)
        loc[i] = toupper(loc[i]);

    double lon = -180.0 + (loc[0] - 'A') * 20.0; // 2 degrees
    double lat = -90.0 + (loc[1] - 'A') * 10.0; // 1 degree

    lon += (loc[2] - '0') * 2.0;
    lat += (loc[3] - '0') * 1.0;

    double lon_size = 2.0;
    double lat_size = 1.0;

    if (strlen(locator) == 6)
    {
        lon += (loc[4] - 'A') * (2.0 / 24.0); // 5 minutes
        lat += (loc[5] - 'A') * (1.0 / 24.0); // 2.5 minutes
        lon_size = 2.0 / 24.0;
        lat_size = 1.0 / 24.0;
    }

    lon += lon_size / 2.0;
    lat += lat_size / 2.0;

    return {lat, lon, true};
}
