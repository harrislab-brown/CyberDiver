#Author-John Antolik
#Description-Create a disk flexure based on zig zag beams.

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
        numBeams = 5
        numBeamSegments = 6

        slotWidth = 0.015 * 2.54  # centimeters, width of the cutout between beams
        stockThickness = 0.015 * 2.54  # centimeters

        outerRadius = 1.75 / 2 * 2.54  # centimeters, beam starts here
        innerRadius = 0.25 / 2 * 2.54  # beam ends at this radius
        outerRingWidth = 5/32 * 2.54  # centimeters
        innerRingWidth = 1/16 * 2.54  # centimeters

        # extrude the basic disk
        sketches = rootComp.sketches
        xyPlane = rootComp.xYConstructionPlane
        diskSketch = sketches.add(xyPlane)
        diskSketch.isVisible = False
        diskSketch.name = 'disk'
        diskSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), innerRadius)
        diskSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), outerRadius)
        rootComp.features.extrudeFeatures.addSimple(diskSketch.profiles.item(1), adsk.core.ValueInput.createByReal(stockThickness), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

        # create a sketch for the beams and for the end fillets
        beamSketch: adsk.fusion.Sketch = sketches.add(xyPlane)
        beamSketch.isVisible = False
        beamSketch.name = 'cutouts'
        endFilletSketch: adsk.fusion.Sketch = sketches.add(xyPlane)
        endFilletSketch.isVisible = False
        endFilletSketch.name = 'fillets'

        # prepare to generate the cutouts
        innerCutoutRadius = innerRadius + innerRingWidth + 0.5 * slotWidth
        outerCutoutRadius = outerRadius - outerRingWidth - 0.5 * slotWidth
        beamWidth = (outerCutoutRadius - innerCutoutRadius - numBeamSegments * slotWidth) / numBeamSegments

        # generate the curves
        for j in range(numBeams):
        
            beamTheta = j / numBeams * 2 * math.pi

            # add the straight radial cutout
            beamSketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(innerCutoutRadius * math.cos(beamTheta), innerCutoutRadius * math.sin(beamTheta), 0), 
                                                               adsk.core.Point3D.create(outerCutoutRadius * math.cos(beamTheta), outerCutoutRadius * math.sin(beamTheta), 0))
            endFilletSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(innerCutoutRadius * math.cos(beamTheta), innerCutoutRadius * math.sin(beamTheta)), 0.5 * slotWidth)
            endFilletSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(outerCutoutRadius * math.cos(beamTheta), outerCutoutRadius * math.sin(beamTheta)), 0.5 * slotWidth)

            for i in range(numBeamSegments + 1):

                beamR = innerCutoutRadius + (outerCutoutRadius - innerCutoutRadius) * i / numBeamSegments

                # pick the connection angle so that the connection is the same width as the rest of the beam
                beamConnectionAngle = 2 * math.asin((beamWidth + slotWidth) / (2 * beamR))

                # add the alternating arc cutouts
                if (i % 2) == 0:
                    beamSketch.sketchCurves.sketchArcs.addByCenterStartSweep(adsk.core.Point3D.create(0, 0, 0), 
                                                                            adsk.core.Point3D.create(beamR * math.cos(beamTheta), beamR * math.sin(beamTheta), 0),
                                                                            2 * math.pi / numBeams - beamConnectionAngle)
                    endFilletSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(beamR * math.cos(beamTheta + 2 * math.pi / numBeams - beamConnectionAngle), 
                                                                                                          beamR * math.sin(beamTheta + 2 * math.pi / numBeams - beamConnectionAngle)), 
                                                                                                          0.5 * slotWidth)
                else:
                    beamSketch.sketchCurves.sketchArcs.addByCenterStartSweep(adsk.core.Point3D.create(0, 0, 0), 
                                                                            adsk.core.Point3D.create(beamR * math.cos(beamTheta + beamConnectionAngle), beamR * math.sin(beamTheta + beamConnectionAngle), 0),
                                                                            2 * math.pi / numBeams - beamConnectionAngle)
                    endFilletSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(beamR * math.cos(beamTheta + beamConnectionAngle), 
                                                                                                          beamR * math.sin(beamTheta + beamConnectionAngle)), 
                                                                                                          0.5 * slotWidth)


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

        # group all of these features
        design.timeline.timelineGroups.add(0, design.timeline.count - 1)

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
