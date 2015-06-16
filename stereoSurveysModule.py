# -*- coding: utf-8 -*-
"""
/***************************************************************************
 stereoSurveys
                                 A QGIS plugin
 Streetview visual surveys
                              -------------------
        begin                : 2015-05-25
        git sha              : $Format:%H$
        copyright            : (C) 2015 by Enrico Ferreguti
        email                : enricofer@gmail.com
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
"""

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtWebKit import *
from qgis.core import *
from qgis.utils import *
from qgis.gui import *
# Initialize Qt resources from file resources.py
import resources_rc
# Import the code for the dialog
from stereoSurveysModule_dialog import stereoSurveysDialog
import os.path
import math
import json
from math import fmod,pi,asin,acos,atan,atan2,sqrt,sin,cos,tan


class stereoSurveys(QgsMapTool):
    """QGIS Plugin Implementation."""

    def __init__(self, iface):
        """Constructor.

        :param iface: An interface instance that will be passed to this class
            which provides the hook by which you can manipulate the QGIS
            application at run time.
        :type iface: QgsInterface
        """
        # Save reference to the QGIS interface
        self.iface = iface
        # initialize plugin directory
        self.plugin_dir = os.path.dirname(__file__)
        # initialize locale
        locale = QSettings().value('locale/userLocale')[0:2]
        locale_path = os.path.join(
            self.plugin_dir,
            'i18n',
            'stereoSurveys_{}.qm'.format(locale))

        if os.path.exists(locale_path):
            self.translator = QTranslator()
            self.translator.load(locale_path)

            if qVersion() > '4.3.3':
                QCoreApplication.installTranslator(self.translator)

        # Create the dialog (after translation) and keep reference
        self.dlg = stereoSurveysDialog()

        # Declare instance attributes
        self.actions = []
        self.menu = self.tr(u'&visual surveys')
        # TODO: We are going to let the user set this up in a future iteration
        self.toolbar = self.iface.addToolBar(u'stereoSurveys')
        self.toolbar.setObjectName(u'stereoSurveys')
        self.canvas = self.iface.mapCanvas()
        QgsMapTool.__init__(self, self.canvas)

    # noinspection PyMethodMayBeStatic
    def tr(self, message):
        """Get the translation for a string using Qt translation API.

        We implement this ourselves since we do not inherit QObject.

        :param message: String for translation.
        :type message: str, QString

        :returns: Translated version of message.
        :rtype: QString
        """
        # noinspection PyTypeChecker,PyArgumentList,PyCallByClass
        return QCoreApplication.translate('stereoSurveys', message)


    def add_action(
        self,
        icon_path,
        text,
        callback,
        enabled_flag=True,
        add_to_menu=True,
        add_to_toolbar=True,
        status_tip=None,
        whats_this=None,
        parent=None):
        """Add a toolbar icon to the toolbar.

        :param icon_path: Path to the icon for this action. Can be a resource
            path (e.g. ':/plugins/foo/bar.png') or a normal file system path.
        :type icon_path: str

        :param text: Text that should be shown in menu items for this action.
        :type text: str

        :param callback: Function to be called when the action is triggered.
        :type callback: function

        :param enabled_flag: A flag indicating if the action should be enabled
            by default. Defaults to True.
        :type enabled_flag: bool

        :param add_to_menu: Flag indicating whether the action should also
            be added to the menu. Defaults to True.
        :type add_to_menu: bool

        :param add_to_toolbar: Flag indicating whether the action should also
            be added to the toolbar. Defaults to True.
        :type add_to_toolbar: bool

        :param status_tip: Optional text to show in a popup when mouse pointer
            hovers over the action.
        :type status_tip: str

        :param parent: Parent widget for the new action. Defaults None.
        :type parent: QWidget

        :param whats_this: Optional text to show in the status bar when the
            mouse pointer hovers over the action.

        :returns: The action that was created. Note that the action is also
            added to self.actions list.
        :rtype: QAction
        """

        icon = QIcon(icon_path)
        action = QAction(icon, text, parent)
        action.triggered.connect(callback)
        action.setEnabled(enabled_flag)

        if status_tip is not None:
            action.setStatusTip(status_tip)

        if whats_this is not None:
            action.setWhatsThis(whats_this)

        if add_to_toolbar:
            self.toolbar.addAction(action)

        if add_to_menu:
            self.iface.addPluginToWebMenu(
                self.menu,
                action)

        self.actions.append(action)

        return action

    def initGui(self):
        """Create the menu entries and toolbar icons inside the QGIS GUI."""
        
        icon_path = os.path.join(self.plugin_dir,"lib","pplane.png")
        self.add_action(
            icon_path,
            text=self.tr(u'Visual surveys'),
            callback=self.run,
            parent=self.iface.mainWindow())
        
        self.wdg = stereoSurveysDialog()
        self.apdockwidget=QDockWidget("stereoSurveys" , self.iface.mainWindow() )
        self.apdockwidget.setObjectName("stereoSurveys")
        self.apdockwidget.setWidget(self.wdg)
        self.apdockwidget.setAllowedAreas(Qt.TopDockWidgetArea | Qt.BottomDockWidgetArea)
        self.iface.addDockWidget( Qt.TopDockWidgetArea, self.apdockwidget)
        self.apdockwidget.update()

        #defaults
        self.actualPointDx = None
        self.actualPointSx = None
        self.actualLocDx = {"lat":0.0,"lon":0.0,"heading":0.0}
        self.actualLocSx = {"lat":0.0,"lon":0.0,"heading":0.0}
        self.pressed = None
        self.updateCalibration(0,0)
        self.googleCameraHeight = 3
        
        #signals
        self.wdg.pushButton.clicked.connect(self.test)
        self.wdg.webViewSx.page().statusBarMessage.connect(self.catchJSeventsSx)
        self.wdg.webViewDx.page().statusBarMessage.connect(self.catchJSeventsDx)
        self.wdg.webViewPlan.page().statusBarMessage.connect(self.catchJSeventsCx)
        self.wdg.correzioneSx.textEdited.connect(self.correggi)
        self.wdg.correzioneDx.textEdited.connect(self.correggi)
        self.wdg.pushButtonDx.clicked.connect(self.setViewDx)
        self.wdg.pushButtonSx.clicked.connect(self.setViewSx)

        #startup
        self.defaults()

    def test(self):
        #js = "this.setMapCenter(%s, %s);" % (45.3995002217,11.8760747755)
        #self.defaults()
        #self.centerControlMap(QgsPoint(11.8760747755,45.3995002217))
        surveyLayer = self.iface.legendInterface().currentLayer()
        if surveyLayer.isValid() and surveyLayer.type() == QgsMapLayer.VectorLayer:
            if surveyLayer.geometryType() == QGis.Point:
                if surveyLayer.isEditable():
                    feat = QgsFeature()
                    geom = QgsGeometry.fromPoint(self.transformToLayerSRS(surveyLayer,self.intersectionPoint))
                    feat.setGeometry(geom)
                    #feat.setAttributes([99])
                    surveyLayer.addFeatures([feat])
                    s = QSettings()
                    attmode = s.value("Qgis/digitizing/disable_enter_attribute_values_dialog", "" )
                    if attmode == 'false':
                        self.iface.openFeatureForm(surveyLayer,feat,True)
                    self.iface.mapCanvas().refresh()
                    surveyLayer.setSelectedFeatures([])
                else:
                    self.iface.messageBar().pushMessage("Warning", "Current layer is not editable", level=QgsMessageBar.WARNING , duration=3)
            else:
                self.iface.messageBar().pushMessage("Warning", "Point layer needed to digitize position", level=QgsMessageBar.WARNING , duration=3)
        else:
            self.iface.messageBar().pushMessage("Warning", "Invalid current layer", level=QgsMessageBar.WARNING , duration=3)

    def ex_centerControlMap(self,loc,zoom = 20):
        js = "this.mapClient.panTo(new google.maps.LatLng(%s, %s));" % (loc.y(),loc.x())
        self.wdg.webViewPlan.page().mainFrame().evaluateJavaScript(js)
        #js = "this.mapClient.setZoom(17);"
        #self.wdg.webViewPlan.page().mainFrame().evaluateJavaScript(js)

    def centerControlMap(self,loc,zoom = 20):
        if self.calibrated:
            js = "this.setCalculatedIntersection(%s, %s);" % (loc.y(),loc.x())
        else:
            js = "this.setMapCenter(%s, %s);" % (loc.y(),loc.x())
        self.wdg.webViewPlan.page().mainFrame().evaluateJavaScript(js)
        #js = "this.mapClient.setZoom(17);"
        #self.wdg.webViewPlan.page().mainFrame().evaluateJavaScript(js)

    def test_centerControlMap(self,loc1,loc2):
        js = "this.setMapintersection(%s, %s,%s, %s,%s, %s);" % (loc1["lon"],loc1["lat"],loc1["heading"],loc2["lon"],loc2["lat"],loc2["heading"])
        self.wdg.webViewPlan.page().mainFrame().evaluateJavaScript(js)
        #js = "this.mapClient.setZoom(17);"
        #self.wdg.webViewPlan.page().mainFrame().evaluateJavaScript(js)
        
    def catchJSeventsSx(self,status):
        try:
            tmpLoc = json.JSONDecoder().decode(status)
        except:
            tmpLoc = None
        if tmpLoc:
            print "SX",tmpLoc
            if tmpLoc["transport"] == "viewPosition":
                self.updateCalibration(0,0)
                self.actualLocSx = tmpLoc
                self.actualPointSx = self.transformToCurrentSRS(QgsPoint(float(self.actualLocSx['lon']),float(self.actualLocSx['lat'])))
                self.wdg.labelLatSx.setText(str(self.actualLocSx['lat']))
                self.wdg.labelLonSx.setText(str(self.actualLocSx['lon']))
                self.wdg.labelHeadingSx.setText(str(self.actualLocSx['heading']))
                self.wdg.labelPhotoHeadingSx.setText(str(self.actualLocSx['photoHeading']))
                #self.actualPointSx = QgsPoint(float(self.actualLocSx['lon']),float(self.actualLocSx['lat']))
                self.updateIntersection()
                print "&&",self.intersectionPoint
                self.wdg.labelPitchSx.setText(str(self.actualLocSx['pitch']))
                distSx = math.sqrt(self.actualPointSx.sqrDist(self.intersectionPoint))
                heightSx = (math.tan( math.radians(self.actualLocSx['pitch']))*distSx)+float(self.wdg.googleCameraHeightSx.text())
                self.wdg.labelDistSx.setText(str(distSx))
                self.wdg.labelHeightSx.setText(str(heightSx))
            elif tmpLoc["transport"] == "viewPov":
                self.actualLocSx = tmpLoc
                self.wdg.labelHeadingSx.setText(str(self.actualLocSx['heading']))
                self.updateIntersection()
                self.wdg.labelPitchSx.setText(str(self.actualLocSx['pitch']))
                distSx = math.sqrt(self.actualPointSx.sqrDist(self.intersectionPoint))
                heightSx = (math.tan( math.radians(self.actualLocSx['pitch']))*distSx)+float(self.wdg.googleCameraHeightSx.text())
                self.wdg.labelDistSx.setText(str(distSx))
                self.wdg.labelHeightSx.setText(str(heightSx))


    def catchJSeventsDx(self,status):
        try:
            tmpLoc = json.JSONDecoder().decode(status)
        except:
            tmpLoc = None
        if tmpLoc:
            print "DX",tmpLoc
            if tmpLoc["transport"] == "viewPosition":
                self.updateCalibration(0,0)
                self.actualLocDx = tmpLoc
                self.actualPointDx = self.transformToCurrentSRS(QgsPoint(float(self.actualLocDx['lon']),float(self.actualLocDx['lat'])))
                self.wdg.labelLatDx.setText(str(self.actualLocDx['lat']))
                self.wdg.labelLonDx.setText(str(self.actualLocDx['lon']))
                self.wdg.labelHeadingDx.setText(str(self.actualLocDx['heading']))
                self.wdg.labelPhotoHeadingDx.setText(str(self.actualLocDx['photoHeading']))
                #self.actualPointDx = QgsPoint(float(self.actualLocDx['lon']),float(self.actualLocDx['lat']))
                self.updateIntersection()
                self.wdg.labelPitchDx.setText(str(self.actualLocDx['pitch']))
                distDx = math.sqrt(self.actualPointDx.sqrDist(self.intersectionPoint))
                heightDx = (math.tan( math.radians(self.actualLocDx['pitch']))*distDx)+float(self.wdg.googleCameraHeightDx.text())
                self.wdg.labelDistDx.setText(str(distDx))
                self.wdg.labelHeightDx.setText(str(heightDx))
            elif tmpLoc["transport"] == "viewPov":
                self.actualLocDx = tmpLoc
                self.wdg.labelHeadingDx.setText(str(self.actualLocDx['heading']))
                self.updateIntersection()
                self.wdg.labelPitchDx.setText(str(self.actualLocDx['pitch']))
                distDx = math.sqrt(self.actualPointDx.sqrDist(self.intersectionPoint))
                heightDx = (math.tan( math.radians(self.actualLocDx['pitch']))*distDx)+float(self.wdg.googleCameraHeightDx.text())
                self.wdg.labelDistDx.setText(str(distDx))
                self.wdg.labelHeightDx.setText(str(heightDx))

    def catchJSeventsCx(self,status):
        try:
            tmpLoc = json.JSONDecoder().decode(status)
        except:
            print "errore finestra centrale"
            tmpLoc = None
        if tmpLoc:
            print "CX",tmpLoc
            if tmpLoc["transport"] == "calibration":
                correctionPoint = self.transformToCurrentSRS(QgsPoint(float(tmpLoc['lon']),float(tmpLoc['lat'])))
                newHeadingSx = self.getTrueHeading(self.actualPointSx,correctionPoint)
                newHeadingDx = self.getTrueHeading(self.actualPointDx,correctionPoint)
		
                self.calibrationSx = self.actualLocSx['heading']-newHeadingSx
                self.calibrationDx = self.actualLocDx['heading']-newHeadingDx

        self.updateCalibration(self.calibrationSx,self.calibrationDx)

                #print "ERROR:",deltaX,deltaY
                #print "NEWHEADINGS:",newHeadingSx,newHeadingDx
                #print "CALIBRATION:",self.actualLocSx['heading']-newHeadingSx,self.actualLocDx['heading']-newHeadingDx

    def updateCalibration(self,sx,sy):
        if sx == 0 and sy == 0:
            self.calibrated = None
            self.wdg.labelCalibrationSx.setText("not calibrated")
            self.wdg.labelCalibrationDx.setText("not calibrated")
            self.calibrationSx = 0
            self.calibrationDx = 0
        else:
            self.calibrated = True
            self.calibrationSx = sx
            self.calibrationDx = sy
            self.wdg.labelCalibrationSx.setText(str(self.calibrationSx))
            self.wdg.labelCalibrationDx.setText(str(self.calibrationDx))

	


    def getTrueHeading(self,p1,p2):
        if (p1.x()==p2.x())&(p1.y()==p2.y()):
            return 0
        else:
            result = math.atan2((p2.x() - p1.x()),(p2.y() - p1.y()))
            result = math.degrees(result)
	    #print "result",result
            if result > 0:
                return result
            else:
                return 360 +  result
                
    def correggi(self,txt):
        try:
            self.updateIntersection()
        except:
            self.iface.messageBar().pushMessage("Warning", "Numeric correction required", level=QgsMessageBar.WARNING , duration=1)

    def updateIntersection(self):
        if self.actualPointDx and self.actualPointSx:
            #p1x = float(self.actualLocSx['lon'])
            #p1y = float(self.actualLocSx['lat'])
            #p2x = float(self.actualLocDx['lon'])
            #p2y = float(self.actualLocDx['lat'])
            p1x = self.actualPointSx.x()
            p1y = self.actualPointSx.y()
            p2x = self.actualPointDx.x()
            p2y = self.actualPointDx.y()
            corrSx = float (self.wdg.correzioneSx.text())
            corrDx = float (self.wdg.correzioneDx.text())
            k1 = math.tan(math.radians(90-self.actualLocSx["heading"]+corrSx+self.calibrationSx))
            k2 = math.tan(math.radians(90-self.actualLocDx["heading"]+corrDx+self.calibrationDx))
            m1 = p1y - p1x*k1
            m2 = p2y - p2x*k2
            self.xInt = (m2-m1)/(k1-k2)
            self.yInt = self.xInt*k1 + m1
            self.yInt2 = self.xInt*k2 + m2
            self.intersectionPoint = QgsPoint(self.xInt,self.yInt)
            self.wdg.labelY.setText(str(self.yInt))
            self.wdg.labelX.setText(str(self.xInt))
            self.intersectionLoc = self.transformToWGS84(self.intersectionPoint)
            self.wdg.labelLat.setText(str(self.intersectionLoc.y()))
            self.wdg.labelLon.setText(str(self.intersectionLoc.x()))
            #self.intersectionLoc = self.intersectionPoint
            #self.centerControlMap(self.actualLocSx,self.actualLocDx)
            self.centerControlMap(self.intersectionLoc)
            self.centerMapCanvasLocations()

    def centerMapCanvasLocations(self):
        print "centerLoc"
        try:
            self.positionDx.reset()
            self.positionSx.reset()
            self.positionInt.reset()
        except:
            pass
        self.positionDx=QgsRubberBand(iface.mapCanvas(),QGis.Point )
        self.positionDx.setWidth( 5 )
        self.positionDx.setIcon(QgsRubberBand.ICON_CIRCLE)
        self.positionDx.setIconSize(6)
        self.positionDx.setColor(Qt.red)
        self.positionSx=QgsRubberBand(iface.mapCanvas(),QGis.Point )
        self.positionSx.setWidth( 5 )
        self.positionSx.setIcon(QgsRubberBand.ICON_CIRCLE)
        self.positionSx.setIconSize(6)
        self.positionSx.setColor(Qt.green)
        self.positionInt=QgsRubberBand(iface.mapCanvas(),QGis.Point )
        self.positionInt.setWidth( 5 )
        self.positionInt.setIcon(QgsRubberBand.ICON_CIRCLE)
        self.positionInt.setIconSize(3)
        self.positionInt.setColor(Qt.yellow)
        #self.positionDx.addPoint(self.transformToCurrentSRS(self.actualPointDx))
        #self.positionSx.addPoint(self.transformToCurrentSRS(self.actualPointSx))
        #self.positionInt.addPoint(self.transformToCurrentSRS(self.intersectionPoint))
        self.positionDx.addPoint(self.actualPointDx)
        self.positionSx.addPoint(self.actualPointSx)
        self.positionInt.addPoint(self.intersectionPoint)

    def transformToWGS84(self, pPoint):
        # transformation from the current SRS to WGS84
        crcMappaCorrente = iface.mapCanvas().mapRenderer().destinationCrs() # get current crs
        crsSrc = crcMappaCorrente
        crsDest = QgsCoordinateReferenceSystem(4326)  # WGS 84
        xform = QgsCoordinateTransform(crsSrc, crsDest)
        return xform.transform(pPoint) # forward transformation: src -> dest

    def transformToCurrentSRS(self, pPoint):
        # transformation from the current SRS to WGS84
        crcMappaCorrente = iface.mapCanvas().mapRenderer().destinationCrs() # get current crs
        crsDest = crcMappaCorrente
        crsSrc = QgsCoordinateReferenceSystem(4326)  # WGS 84
        xform = QgsCoordinateTransform(crsSrc, crsDest)
        return xform.transform(pPoint) # forward transformation: src -> dest

    def transformToLayerSRS(self,layer, pPoint):
        # transformation from project SRS the provided layer SRS
        crsDest = layer.crs() # get layer crs
        crsSrc = iface.mapCanvas().mapRenderer().destinationCrs()  # project srs
        xform = QgsCoordinateTransform(crsSrc, crsDest)
        return xform.transform(pPoint) # forward transformation: src -> dest

    def unload(self):
        """Removes the plugin menu item and icon from QGIS GUI."""
        for action in self.actions:
            self.iface.removePluginWebMenu(
                self.tr(u'&visual surveys'),
                action)
            self.iface.removeToolBarIcon(action)
        # remove the toolbar
        del self.toolbar
        try:
            self.positionDx.reset()
            self.positionSx.reset()
            self.positionInt.reset()
        except:
            pass

    def defaults(self):
        swUrlSx = "qrc:///plugins/stereoSurveys/lib/sv.html?lat=45.3993885731&long=11.875303141&width=400&height=250&heading=131"
        swUrlDx = "qrc:///plugins/stereoSurveys/lib/sv.html?lat=45.399895797&long=11.8765856845&width=400&height=250&heading=194"
        mapPlanUrl = "qrc:///plugins/stereoSurveys/lib/gm.html?width=150&height=150&zoom=20"
        self.wdg.webViewSx.load(QUrl(swUrlSx))
        self.wdg.webViewDx.load(QUrl(swUrlDx))
        self.wdg.webViewPlan.load(QUrl(mapPlanUrl))
        self.calibrated = None

    def canvasPressEvent(self, event):
        # Press event handler inherited from QgsMapTool used to store the given location in WGS84 long/lat
        self.pressed=True
        self.pressx = event.pos().x()
        self.pressy = event.pos().y()
        self.movex = event.pos().x()
        self.movey = event.pos().y()
        self.highlight=QgsRubberBand(iface.mapCanvas(),QGis.Line )
        self.highlight.setColor(Qt.yellow)
        self.highlight.setWidth(5)
        #print "x:",self.pressx," y:",self.pressy
        self.PressedPoint = self.canvas.getCoordinateTransform().toMapCoordinates(self.pressx, self.pressy)
        #print self.PressedPoint.x(),self.PressedPoint.y()
        self.pointWgs84 = self.transformToWGS84(self.PressedPoint)

    def canvasMoveEvent(self, event):
        # Moved event handler inherited from QgsMapTool needed to highlight the direction that is giving by the user
        if self.pressed:
            #print "canvasMoveEvent"
            x = event.pos().x()
            y = event.pos().y()
            movedPoint = self.canvas.getCoordinateTransform().toMapCoordinates(x, y)
            self.highlight.reset()
            self.highlight.addPoint(self.PressedPoint)
            self.highlight.addPoint(movedPoint)


    def canvasReleaseEvent(self, event):
        # Release event handler inherited from QgsMapTool needed to calculate heading
        self.pressed=None
        self.highlight.reset()
        self.releasedx = event.pos().x()
        self.releasedy = event.pos().y()
        self.releasedPoint = self.canvas.getCoordinateTransform().toMapCoordinates(self.releasedx,self.releasedy)
        #print "x:",self.releasedx," y:",self.releasedy
        if (self.releasedx==self.pressx)&(self.releasedy==self.pressy):
            self.heading=0
            result=0
        else:
            result = math.atan2((self.releasedx - self.pressx),(self.releasedy - self.pressy))
            result = math.degrees(result)
            if result > 0:
                self.heading =  180 - result
            else:
                self.heading = 360 - (180 + result)
        print ">",self.heading
        print ">>",self.getTrueHeading(self.PressedPoint,self.releasedPoint)
        print ">>>",self.getTrueHeading(self.releasedPoint,self.PressedPoint)
        self.wdg.pushButtonSx.setDown(False)
        self.wdg.pushButtonSx.setDown(False)

        #self.heading = math.trunc(self.heading)
        if self.target == "dx":
            self.viewWidth = self.dlg.webViewDx.width()
            self.viewHeight = self.dlg.webViewDx.height()
            self.gswUrl = "qrc:///plugins/stereoSurveys/lib/sv.html?lat="+str(self.pointWgs84.y())+"&long="+str(self.pointWgs84.x())+"&width="+str(self.viewWidth)+"&height="+str(self.viewHeight)+"&heading="+str(self.heading)
            print "dx",self.gswUrl
            self.wdg.webViewDx.load(QUrl(self.gswUrl))
        if self.target == "sx":
            self.viewWidth = self.dlg.webViewSx.width()
            self.viewHeight = self.dlg.webViewSx.height()
            self.gswUrl = "qrc:///plugins/stereoSurveys/lib/sv.html?lat="+str(self.pointWgs84.y())+"&long="+str(self.pointWgs84.x())+"&width="+str(self.viewWidth)+"&height="+str(self.viewHeight)+"&heading="+str(self.heading)
            print "sx",self.gswUrl
            self.wdg.webViewSx.load(QUrl(self.gswUrl))

    def setViewDx(self):
        gsvMessage="Click on map and drag the cursor to the desired direction to display Google Street View in right panel"
        iface.mainWindow().statusBar().showMessage(gsvMessage)
        self.target = "dx"
        self.wdg.pushButtonSx.setDown(True)
        self.canvas.setMapTool(self)

    def setViewSx(self):
        gsvMessage="Click on map and drag the cursor to the desired direction to display Google Street View in left panel"
        iface.mainWindow().statusBar().showMessage(gsvMessage)
        self.target = "sx"
        self.wdg.pushButtonSx.setDown(True)
        self.canvas.setMapTool(self)

    def run(self):
        # called by click on toolbar icon
        if self.apdockwidget.isVisible():
            self.apdockwidget.hide()
        else:
            self.apdockwidget.show()
            #self.surveyLayer = QgsVectorLayer("Point?crs="+self.iface.mapCanvas().mapRenderer().destinationCrs().toWkt(), "SurveyLayer", "memory")
            #QgsMapLayerRegistry.instance().addMapLayer(self.surveyLayer)
