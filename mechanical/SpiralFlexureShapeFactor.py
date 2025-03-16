#Author-John Antolik
#Description-Create a disk flexure based on Archimedes spiral beams. Thin the beams in the middle according to a shape factor

import adsk.core, adsk.fusion, adsk.cam, traceback, math

def run(context):
    ui = None
    try:
        # boilerplate API stuff
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct
        rootComp = design.rootComponent

        # curve parameters
        splinePoints = 100  # discretize the curve with this many points, accuracy vs performance tradeoff
        turnAngle = 540 * math.pi / 180.0  # radians, what angle does the beam turn through between inner and outer radius
        numBeams = 3
        shapeFactor = 0.075  # how much to thin the beams in the middle

        slotWidth = 0.015 * 2.54  # centimeters, width of the cutout between beams
        stockThickness = 0.015 * 2.54  # centimeters

        outerRadius = 1.75 / 2 * 2.54  # centimeters, beam starts here
        innerRadius = 0.25 / 2 * 2.54  # beam ends at this radius
        outerRingWidth = 5/32 * 2.54  # centimeters
        innerRingWidth = 1/16 * 2.54  # centimeters

        # create a sketch for the beams and for the end fillets
        sketches = rootComp.sketches
        xyPlane = rootComp.xYConstructionPlane
        beamSketch = sketches.add(xyPlane)
        beamSketch.isVisible = False
        beamSketch.name = 'spirals'
        endFilletSketch = sketches.add(xyPlane)
        endFilletSketch.isVisible = False
        endFilletSketch.name = 'end fillets'

        # prepare to store the points of the equation driven curve
        points = adsk.core.ObjectCollection.create()
        innerCutoutRadius = innerRadius + innerRingWidth + 0.5 * slotWidth
        outerCutoutRadius = outerRadius - outerRingWidth - 0.5 * slotWidth

        # generate the curves
        for j in range(numBeams):
        
            points.clear()
            startTheta = j / numBeams * 2 * math.pi

            for i in range(splinePoints + 1):

                theta = startTheta + i / splinePoints * turnAngle 
                r = innerCutoutRadius + (outerCutoutRadius - innerCutoutRadius) * ((theta - startTheta) / turnAngle + shapeFactor * math.sin(2 * math.pi * (theta - startTheta) / turnAngle))

                xCoord = r * math.cos(theta)
                yCoord = r * math.sin(theta)

                points.add(adsk.core.Point3D.create(xCoord, yCoord, 0))

                # generate the circles to round the ends of the cutouts
                if i == 0 or i == splinePoints:
                    endFilletSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(xCoord, yCoord, 0), 0.5 * slotWidth)

            # generate spline curve from points
            beamSketch.sketchCurves.sketchFittedSplines.add(points)

        # extrude the basic disk
        diskSketch = sketches.add(xyPlane)
        diskSketch.isVisible = False
        diskSketch.name = 'disk'
        diskSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), innerRadius)
        diskSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), outerRadius)
        rootComp.features.extrudeFeatures.addSimple(diskSketch.profiles.item(1), adsk.core.ValueInput.createByReal(stockThickness), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # extrude the beams
        pros = []
        objs = adsk.core.ObjectCollection.create()
        for crv in beamSketch.sketchCurves:
            objs.clear()
            objs.add(crv)
            pros.append(rootComp.createOpenProfile(objs, False))

        for profile in pros:
            thinExtrude(profile, slotWidth, stockThickness, adsk.fusion.ThinExtrudeWallLocation.Center, adsk.fusion.FeatureOperations.CutFeatureOperation)

        # extrude the beam end fillets
        for profile in endFilletSketch.profiles:
             rootComp.features.extrudeFeatures.addSimple(profile, adsk.core.ValueInput.createByReal(stockThickness), adsk.fusion.FeatureOperations.CutFeatureOperation)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def thinExtrude(profile, thickness, distance, side: adsk.fusion.ThinExtrudeWallLocation, operation: adsk.fusion.FeatureOperations):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct
        rootComp = design.rootComponent

        # get extrude features and define the extrude input
        extrudes = rootComp.features.extrudeFeatures
        extrudeInput = extrudes.createInput(profile, operation)
        wallThickness = adsk.core.ValueInput.createByReal(thickness)
        extrudeInput.setThinExtrude(side, wallThickness)
        extrudeDistance = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(distance))
        extrudeInput.setOneSideExtent(extrudeDistance, adsk.fusion.ExtentDirections.PositiveExtentDirection)

        # create the feature
        return extrudes.add(extrudeInput)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
