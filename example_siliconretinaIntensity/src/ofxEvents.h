//
//  ofxEvents.h
//  example_siliconretinaIntensity
//
//  Created by Federico Corradi on 27.05.17.
//
//

#ifndef ofxEvents_h
#define ofxEvents_h

#pragma once

#define STRINGIFY(A) #A

#include "ofMain.h"
#include "ofxFXObject.h"

class ofxEvents : public ofxFXObject {
public:
    ofxEvents(){
        passes = 1;
        internalFormat = GL_RGBA32F ;
        
        dumping = 0.5;
        
        fragmentShader = STRINGIFY(uniform sampler2DRect backbuffer;   // previus buffer
                                   uniform sampler2DRect tex0;         // actual buffer
                                   
                                   uniform float dumping;
                                   vec2 offset[12];
                                   
                                   void main(){
                                       vec2 st = gl_TexCoord[0].st;
                                       offset[0] = vec2(-1.0, 0.0);
                                       offset[1] = vec2(1.0, 0.0);
                                       offset[2] = vec2(0.0, 1.0);
                                       offset[3] = vec2(0.0, -1.0);
                                       
                                       offset[4] = vec2(-2.0, 0.0);
                                       offset[5] = vec2(2.0, 0.0);
                                       offset[6] = vec2(0.0, 2.0);
                                       offset[7] = vec2(0.0, -2.0);
                                       
                                       offset[8] = vec2(-1.0, 1.0);
                                       offset[9] = vec2(1.0, 1.0);
                                       offset[10] = vec2(1.0, -1.0);
                                       offset[11] = vec2(-1.0, -1.0);
                                       
                                       //  Grab the information arround the active pixel
                                       //
                                       //           [7]
                                       //
                                       //      [8]  [3]  [9]
                                       //
                                       // [4]  [0]  st   [1] [5]
                                       //
                                       //      [11] [2]  [10]
                                       //
                                       //           [6]
                                       
                                       
                                       vec3 sum = vec3(0.0, 0.0, 0.0);
                                       
                                       for (int i = 0; i < 8 ; i++){
                                           sum += texture2DRect(tex0, st + offset[i]).rgb;
                                       }
                                       
                                       sum = (sum / 4.0) - (texture2DRect(backbuffer, st).rgb - texture2DRect(tex0, st).rgb);
                                       
                                       sum *= dumping;
                                       
                                       
                                       gl_FragColor = vec4(sum, 1.0);
                                   } );
    }
    
    void begin() {
        ofPushStyle();
        ofPushMatrix();
        pingPong.src->begin();
    }
    
    void end() {
        pingPong.src->end();
        ofPopMatrix();
        ofPopStyle();
    }
    
    void update(){
        // Calculate the difference between buffers and spread the waving
        textures[0].begin();
        shader.begin();
        shader.setUniformTexture("backbuffer", pingPong.dst->getTexture(), 0);
        shader.setUniformTexture("tex0", pingPong.src->getTexture(), 1);
        shader.setUniform1f("dumping", (float)dumping );
        renderFrame();
        shader.end();
        textures[0].end();
        
        // TODO: improve this, it's almost non-sense
        pingPong.dst->begin();
        textures[0].draw(0, 0);
        pingPong.dst->end();
        
        pingPong.swap();
    }
    
    float   dumping;
};


#endif /* ofxEvents_h */
