
#include <stdio.h>
#include "contour-combiners.h"

#include "arithmetics.hpp"

namespace msdfgen {

static void initDistance(double &distance) {
    distance = SignedDistance::INFINITE.distance;
}

static void initDistance(MultiDistance &distance) {
    distance.r = SignedDistance::INFINITE.distance;
    distance.g = SignedDistance::INFINITE.distance;
    distance.b = SignedDistance::INFINITE.distance;
}

static double resolveDistance(double distance) {
    return distance;
}

static double resolveDistance(const MultiDistance &distance) {
    return median(distance.r, distance.g, distance.b);
}
static void dumpDistance(double distance) {
    return;
}

static void dumpDistance(const MultiDistance &distance) {
    // printf("+dump  \t%.2f,\t%.2f,\t%.2f\n", distance.r, distance.g, distance.b);
    return;
}

template <class EdgeSelector>
SimpleContourCombiner<EdgeSelector>::SimpleContourCombiner(const Shape &shape) { }

template <class EdgeSelector>
void SimpleContourCombiner<EdgeSelector>::reset(const Point2 &p) {
    shapeEdgeSelector = EdgeSelector(p);
}

template <class EdgeSelector>
void SimpleContourCombiner<EdgeSelector>::setContourEdge(int i, const EdgeSelector &edgeSelector) {
    shapeEdgeSelector.merge(edgeSelector);
}

template <class EdgeSelector>
typename SimpleContourCombiner<EdgeSelector>::DistanceType SimpleContourCombiner<EdgeSelector>::distance() const {
    return shapeEdgeSelector.distance();
}

template class SimpleContourCombiner<TrueDistanceSelector>;
template class SimpleContourCombiner<PseudoDistanceSelector>;
template class SimpleContourCombiner<MultiDistanceSelector>;

template <class EdgeSelector>
OverlappingContourCombiner<EdgeSelector>::OverlappingContourCombiner(const Shape &shape) {
    windings.reserve(shape.contours.size());
    for (std::vector<Contour>::const_iterator contour = shape.contours.begin(); contour != shape.contours.end(); ++contour)
        windings.push_back(contour->winding());
    edgeSelectors.resize(shape.contours.size());
}

template <class EdgeSelector>
void OverlappingContourCombiner<EdgeSelector>::reset(const Point2 &p) {
    shapeEdgeSelector = EdgeSelector(p);
    innerEdgeSelector = EdgeSelector(p);
    outerEdgeSelector = EdgeSelector(p);
}

    #include <stdio.h>
template <class EdgeSelector>
void OverlappingContourCombiner<EdgeSelector>::setContourEdge(int i, const EdgeSelector &edgeSelector) {
    
    edgeSelector.dump();

    DistanceType edgeDistance = edgeSelector.distance();
    // MultiDistance *m = dynamic_cast<MultiDistance *>(&edgeDistance);
    
        printf("--> d: %.2f\n", resolveDistance(edgeDistance));
    static int debug = 1;
    if (debug) {
        dumpDistance(edgeDistance);
        // printf("->  edgeDistance: %.2f, %.2f, %.2f\n", edgeDistance.distance, edgeDistance.g.x, edgeDistance.b.x);
        debug--;
    }
    // printf("--> winding: %d\n", windings[i]);
    edgeSelectors[i] = edgeSelector;
    shapeEdgeSelector.merge(edgeSelector);
    if (windings[i] > 0 && resolveDistance(edgeDistance) >= 0)
        innerEdgeSelector.merge(edgeSelector);
    if (windings[i] < 0 && resolveDistance(edgeDistance) <= 0)
        outerEdgeSelector.merge(edgeSelector);
}

template <class EdgeSelector>
typename OverlappingContourCombiner<EdgeSelector>::DistanceType OverlappingContourCombiner<EdgeSelector>::distance() const {
    DistanceType shapeDistance = shapeEdgeSelector.distance();
    DistanceType innerDistance = innerEdgeSelector.distance();
    DistanceType outerDistance = outerEdgeSelector.distance();
    double innerScalarDistance = resolveDistance(innerDistance);
    double outerScalarDistance = resolveDistance(outerDistance);
    printf("--> inner: %.2e, outer: %.2e\n", innerScalarDistance, outerScalarDistance);
    DistanceType distance;
    initDistance(distance);
    int contourCount = (int) windings.size();
    printf("--> contour count: %d\n", contourCount);

    int winding = 0;
    if (innerScalarDistance >= 0 && fabs(innerScalarDistance) <= fabs(outerScalarDistance)) {
        distance = innerDistance;
        winding = 1;
        for (int i = 0; i < contourCount; ++i)
            if (windings[i] > 0) {
                DistanceType contourDistance = edgeSelectors[i].distance();
                if (fabs(resolveDistance(contourDistance)) < fabs(outerScalarDistance) 
                    && resolveDistance(contourDistance) > resolveDistance(distance))
                    distance = contourDistance;
            }
    } else if (outerScalarDistance <= 0 && fabs(outerScalarDistance) < fabs(innerScalarDistance)) {
        distance = outerDistance;
        winding = -1;
        for (int i = 0; i < contourCount; ++i)
            if (windings[i] < 0) {
                DistanceType contourDistance = edgeSelectors[i].distance();
                if (fabs(resolveDistance(contourDistance)) < fabs(innerScalarDistance) 
                    && resolveDistance(contourDistance) < resolveDistance(distance))
                    distance = contourDistance;
            }
    } else
        return shapeDistance;

    for (int i = 0; i < contourCount; ++i)
        if (windings[i] != winding) {
            DistanceType contourDistance = edgeSelectors[i].distance();
            if (resolveDistance(contourDistance)*resolveDistance(distance) >= 0 
                && fabs(resolveDistance(contourDistance)) < fabs(resolveDistance(distance)))
                distance = contourDistance;
        }
    if (resolveDistance(distance) == resolveDistance(shapeDistance))
        distance = shapeDistance;
    return distance;
}

template class OverlappingContourCombiner<TrueDistanceSelector>;
template class OverlappingContourCombiner<PseudoDistanceSelector>;
template class OverlappingContourCombiner<MultiDistanceSelector>;

}
