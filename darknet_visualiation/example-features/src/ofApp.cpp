#include "ofApp.h"

void ofApp::setup()
{
    ofSetWindowShape(1280, 720);
    ofSetBackgroundAuto(false);
    
    std::string cfgfile = ofToDataPath( "cfg/yolo-obj-webCam.cfg" );
    std::string weightfile = ofToDataPath( "yolo-obj-1200.weights" );
    std::string nameslist = ofToDataPath( "cfg/obj.names" );
    
    darknet.init( cfgfile, weightfile, nameslist );

	inputMode = 1;
    
    numClassifications = 10;
	if( inputMode == 0 ) {
        setSourceWebcam();
	}
	else {
		setSourceImage(ofToDataPath("IMAG3762.JPEG") );
	}
}

void ofApp::update() {
    if (inputMode == 0 && darknet.isLoaded()) {
        grab.update();
        if (grab.isFrameNew()) {
            classifications = darknet.classify(grab.getPixels(), numClassifications);
        }
	}
	else {
		classifications = darknet.classify( pic.getPixels(), numClassifications );
	}
}

void ofApp::draw() {
    ofBackground(100);
    
    if (!darknet.isLoaded()) {
        ofDrawBitmapString("Can't find network file! Check to make sure it's in your data folder.", 20, 20);
        return;
    }
    
    if (inputMode == 0) {   // webcam
        grab.draw(0, 0, 320, 240);
    }
    else {  // image
        pic.draw(0, 0, 320, pic.getHeight() * 320 / pic.getWidth());
    }
    
    // draw menu
    ofNoFill();
    inputMode == 0 ? ofSetColor(0, 255, 0) : ofSetColor(255);
    ofDrawRectangle(10, 315, 200, 20);
    ofDrawBitmapString("Webcam", 15, 332);
    inputMode == 1 ? ofSetColor(0, 255, 0) : ofSetColor(255);
    ofDrawRectangle(10, 338, 200, 20);
    ofDrawBitmapString("Load picture", 15, 355);
    
    vector<string> layerNames = darknet.getLayerNames();
    for (int i=0; i<layerNames.size()-1; i++) { // skip final cost "layer"
        int row = i % 13;
        int col = floor(i/13);
        i == layer ? ofSetColor(0, 255, 0) : ofSetColor(255);
        ofDrawRectangle(10 + 105 * col, 375+23*row, 100, 20);
        ofDrawBitmapString(layerNames[i], 15 + 105 * col, 392+23*row);
    }
    ofSetColor(0, 255, 0);
    ofDrawBitmapString("drag feature maps to scroll", 5, ofGetHeight()-5);
    
    // draw feature maps
    if (layerNames[layer] == "Softmax 0") {
        drawClassifications();
    }
    else {
        drawFeatureMaps();
    }
}

void ofApp::drawClassifications() {
    ofSetColor(255);
    ofDrawBitmapString("Classifications ("+ofToString(ofGetFrameRate(),0,1)+" fps)", 325, 18);
    
    ofTranslate(325, 0);
    for (int i=0; i<classifications.size(); i++) {
        ofTranslate(0, 22);
        ofFill();
        ofSetColor(0);
        ofDrawRectangle(0, 0, 200, 18);
        ofSetColor(ofColor::green);
        ofDrawRectangle(0, 0, 200 * classifications[i].probability, 18);
        ofSetColor(255);
        ofDrawBitmapStringHighlight(classifications[i].label, 210, 14);
    }
}

void ofApp::drawFeatureMaps() {
    maps = darknet.getFeatureMaps(layer);

	ofSetColor(255);
    int w = darknet.getNetwork().layers[layer].out_w;   // width of output volume
    int h = darknet.getNetwork().layers[layer].out_h;   // height of output volume
    int n = darknet.getNetwork().layers[layer].out_c;   // depth of output volume
    string msg = "Layer #"+ofToString(layer)+": "+darknet.getLayerNames()[layer]+", output volume size (depth, height, width) = "+ofToString(n)+" x "+ofToString(h)+" x "+ofToString(w);
    ofDrawBitmapString(msg, 325, 18);
    
	int noItemsPerColumn = 4;
	int imageMargin = 5;
	int imageWidth = 150;
	int imageHeight = 150;
	int imageWidthAndMargin = imageWidth + imageMargin;
	int imageHeightAndMargin = imageHeight + imageMargin;
    ofTranslate(325, 25);
    maxPageLength = ceil(maps.size() / noItemsPerColumn) * 105;
    ofImage img;
    img.getTexture().setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
    for (int m=0; m<maps.size(); m++) {
        maps[m].getImage(img);
        ofPushMatrix();
        ofTranslate((m%noItemsPerColumn)*imageWidthAndMargin,
                    floor(m/ noItemsPerColumn)*imageHeightAndMargin - scroll);
        img.draw(0, 0, imageWidth, imageHeight);
        ofPopMatrix();
    }
    
    // draw highlighted
    if (highlighted != -1 && highlighted < maps.size()) {
        activations map = maps[highlighted];
        map.getImage(img);
        ofPushMatrix();
        ofTranslate(ofClamp((highlighted % noItemsPerColumn)*imageWidthAndMargin - imageWidth/2, 0, ofGetWidth()- 2*(imageWidth + 2* imageMargin)),
                    ofClamp(floor(highlighted/ noItemsPerColumn)*imageHeightAndMargin - scroll - imageHeight / 2,
                            20, ofGetHeight()- 2 * (imageHeight + 2 * imageMargin)));
        ofSetColor(0, 255, 0);
        ofFill();
        ofDrawRectangle(-10, -25, imageWidth*2 + 20, 2* imageHeight + 25);
        ofSetColor(0);
        ofDrawBitmapString(darknet.getLayerNames()[layer]+" - "+ofToString(highlighted), 0, -2);
        ofSetColor(255);
        img.draw(0, 0, imageWidth * 2, imageHeight * 2);
        ofPopMatrix();
    }
}

void ofApp::keyPressed(int key) {
}

void ofApp::mouseMoved(int x, int y) {
    for (int i=0; i<8; i++) {
        for (int j=0; j<ceil(maps.size()/8); j++) {
            if (ofRectangle(325 + 105*i, 25 + 105*j - scroll, 100, 100).inside(x, y)) {
                highlighted = i + 8 * j;
                return;
            }
        }
    }
    highlighted = -1;
}

void ofApp::mouseScrolled(ofMouseEventArgs &evt) {
    scroll = ofClamp(scroll - evt.scrollY, 0, maxPageLength);
}

void ofApp::mouseDragged(int x, int y, int button) {

}

void ofApp::setSourceWebcam() {
    inputMode = 0;
    if (!grab.isInitialized()) {
        grab.initGrabber( 640, 480 );
    }
}

void ofApp::setSourceImage(string path) {
    inputMode = 1;
    pic.load(path);
    classifications = darknet.classify(pic.getPixels(), numClassifications);
    if (grab.isInitialized()) {
        grab.close();
    }
}

void ofApp::mousePressed(int x, int y, int button) {
    if (ofRectangle(10, 315, 200, 20).inside(x, y)) {
        setSourceWebcam();
    }
    else if (ofRectangle(10, 338, 200, 20).inside(x, y)) {
        ofFileDialogResult result = ofSystemLoadDialog("Select an image");
        if (result.bSuccess) {
            setSourceImage(result.filePath);
        }
    }
    else {
        int numLayers = darknet.getLayerNames().size()-1;   // skip final cost "layer"
        for (int i=0; i<numLayers; i++) {
            int row = i % 13;
            int col = floor(i/13);
            
            if (ofRectangle(10 + 105*col, 375+23*row, 100, 20).inside(x, y)) {
                layer = i;
                scroll = 0;
                return;
            }
        }
    }
}
