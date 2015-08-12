//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Sebastien Grange
    \version   3.0.0 $Rev: 1243 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include <iostream>
#include <iomanip>
#include <ostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <cstring>
using namespace std;
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// header file printer
int writeHeader(cImage &img, string  imagename, string  filename)
{
    unsigned int   width  = img.getWidth();
    unsigned int   height = img.getHeight();
    unsigned int   bpp    = img.getBytesPerPixel();
    unsigned int   format = img.getFormat();
    int            bcount = width*height*bpp;
    int            ptr    = 0;
    unsigned char *data   = img.getData();

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing \"" << imagename << "\"." << endl;
    out << endl;
    out << "    Automatically generated using CHAI 3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert imagename to root name
    imagename = imagename.substr(imagename.find_last_of('/')+1, imagename.length());
    transform(imagename.begin(), imagename.end(),imagename.begin(), ::toupper);
    replace(imagename.begin(), imagename.end(), '.', '_');
    replace(imagename.begin(), imagename.end(), ' ', '_');
    replace(imagename.begin(), imagename.end(), '\'', '_');
    replace(imagename.begin(), imagename.end(), '-', '_');

    // convert filename to root name
    filename = filename.substr(filename.find_last_of('/')+1, filename.length());
    filename = filename.substr(0, filename.find_first_of('.'));
    transform(filename.begin(), filename.end(),filename.begin(), ::toupper);
    replace(filename.begin(), filename.end(), '.', '_');
    replace(filename.begin(), filename.end(), ' ', '_');
    replace(filename.begin(), filename.end(), '\'', '_');
    replace(filename.begin(), filename.end(), '-', '_');

    // avoid conflicts
    out << endl;
    out << "#ifndef " << filename << "H" << endl;
    out << "#define " << filename << "H" << endl;
    out << endl << endl << endl;

    // include cImage header (required by convenience functions)
    out << "#include \"graphics/CImage.h\"" << endl << endl;

    // print image info
    out << "const unsigned int " << filename << "_SIZE   = " << bcount << ";" << endl;
    out << "const unsigned int " << filename << "_BPP    = " << bpp    << ";" << endl;
    out << "const unsigned int " << filename << "_WIDTH  = " << width  << ";" << endl;
    out << "const unsigned int " << filename << "_HEIGHT = " << height << ";" << endl;
    out << "const unsigned int " << filename << "_FORMAT = " << format << ";" << "    // ";
    switch (format)
    {
        case GL_RGB:  out << "GL_RGB";  break;
        case GL_RGBA: out << "GL_RGBA"; break;
    }
    out << endl << endl;

    // print image payload
    out << "const unsigned char " << filename << "_BYTEARRAY[] =" << endl;
    out << "{";
    for (unsigned int j=0; j<height; j++)
    {
        out << endl << "    ";
        for (unsigned int i=0; i<width; i++)
        {
            for (unsigned int h=0; h<bpp; h++)
            {
                out << "0x" << hex << setw(2) << setfill('0') << (int)(data[ptr++]);
                if (ptr < bcount) out << ", ";
            }
        }
    }
    out << endl << "};" << endl << endl << endl;

    // convenience function: allocate image
    out << "inline cImage *NEW_" << filename << "()" << endl;
    out << "{" << endl;
    out << "    cImage        *img       = new cImage(" << filename << "_WIDTH, " << filename << "_HEIGHT, " << filename << "_FORMAT);" << endl;
    out << "    unsigned char *bytearray = new unsigned char[" << filename << "_SIZE];" << endl;
    out << endl;
    out << "    memcpy(bytearray, " << filename << "_BYTEARRAY, " << filename << "_SIZE);" << endl;
    out << "    img->setData(bytearray, " << filename << "_SIZE, true);" << endl;
    out << endl;
    out << "    return img;" << endl;
    out << "}" << endl << endl << endl;

    // tidy up
    out << endl;
    out << "#endif" << endl;

    out.close();

    return 0;
}


// PNG header file printer
int writeHeaderPNG(cImage &img, string  imagename, string  filename)
{
    const int lineWidth = 256;

    unsigned int   width  = img.getWidth();
    unsigned int   height = img.getHeight();
    unsigned int   bpp    = img.getBytesPerPixel();
    unsigned int   format = img.getFormat();
    int            bcount = width*height*bpp;
    unsigned int   ptr    = 0;
    unsigned char *data   = img.getData();

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing \"" << imagename << "\"." << endl;
    out << endl;
    out << "    Automatically generated using CHAI 3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert imagename to root name
    imagename = imagename.substr(imagename.find_last_of('/')+1, imagename.length());
    transform(imagename.begin(), imagename.end(),imagename.begin(), ::toupper);
    replace(imagename.begin(), imagename.end(), '.', '_');
    replace(imagename.begin(), imagename.end(), ' ', '_');
    replace(imagename.begin(), imagename.end(), '\'', '_');
    replace(imagename.begin(), imagename.end(), '-', '_');

    // convert filename to root name
    filename = filename.substr(filename.find_last_of('/')+1, filename.length());
    filename = filename.substr(0, filename.find_first_of('.'));
    transform(filename.begin(), filename.end(),filename.begin(), ::toupper);
    replace(filename.begin(), filename.end(), '.', '_');
    replace(filename.begin(), filename.end(), ' ', '_');
    replace(filename.begin(), filename.end(), '\'', '_');
    replace(filename.begin(), filename.end(), '-', '_');

    // convert image to PNG
    unsigned char *buffer;
    unsigned int   len;
    if (!cSavePNG(&img, &buffer, &len))
    {
        cout << "PNG compression failed" << endl;
        return -1;
    }

    // avoid conflicts
    out << endl;
    out << "#ifndef " << filename << "H" << endl;
    out << "#define " << filename << "H" << endl;
    out << endl << endl << endl;

    // include cImage header (required by convenience functions)
    out << "#include \"graphics/CImage.h\"" << endl;
    out << "#include \"files/CFileImagePNG.h\"" << endl << endl;

    // print image payload
    out << "const unsigned char " << filename << "_BYTEARRAY[] =" << endl;
    out << "{";
    for (unsigned int s=0; s<len; s++)
    {
        if (s%lineWidth == 0) out << endl << "\t";
        out << "0x" << hex << setw(2) << setfill('0') << (int)(((unsigned char*)buffer)[ptr++]);
        if (ptr < len) out << ", ";
    }
    out << endl << "};" << endl << endl << endl;

    // convenience function: allocate image
    out << "inline cImage *NEW_" << filename << "()" << endl;
    out << "{" << endl;
    out << "    cImage *img = new cImage();" << endl;
    out << "    cLoadPNG(img, " << filename << "_BYTEARRAY, sizeof(" << filename << "_BYTEARRAY));" << endl;
    out << "    return img;" << endl;
    out << "}" << endl << endl << endl;

    // tidy up
    out << endl;
    out << "#endif" << endl;

    out.close();

    img.saveToFile("test.png");

    return 0;
}


// simple usage printer
int usage()
{
    cout << endl << "03-cimage [-c] image.{bmp|gif|jpg|png|raw} [-o header.h]" << endl;
    cout << "\t-c\tuse PNG compression to store image in header" << endl;
    cout << "\t-o\tspecify header filename" << endl;
    cout << "\t-h\tdisplay this message" << endl << endl;

    return -1;
}


//===========================================================================
/*
    DEMO:    03-cimage.cpp

    This example takes an image in any CHAI3D supported format and produces
    a C/C++ compatible header containing the image data and geometry. This
    allows programmers to easily embed images into their executables.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    cImage img;
    string imagename;
    string filename;
    bool   compress = false;

    // process arguments
    if (argc < 2) return usage();
    for (int i=1; i<argc; i++)
    {
        if (argv[i][0] != '-') {
            if (imagename.length() > 0) return usage();
            imagename = string(argv[i]);
            filename  = imagename;
            filename.replace (imagename.find_last_of('.'), 4, ".h");
        }
        else switch (argv[i][1]) {
            case 'h':
                return usage ();
            case 'o':
                if ((i < argc) && (argv[i+1][0] != '-')) {
                    i++;
                    filename = string(argv[i]);
                }
                else return usage ();
                break;
            case 'c':
                compress = true;
                break;
            default:
                return usage ();
        }
    }

    // pretty message
    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Image Converter" << endl;
    cout << "Copyright 2003-2014" << endl;
    cout << "-----------------------------------" << endl;
    cout << endl;

    // report action
    cout << "converting " << imagename << " to " << filename << "..." << endl;

    // read image
    if (!img.loadFromFile (imagename))
    {
        cout << "error: cannot load image file " << imagename << endl;
    }
    else {
        cout << "image load succeeded" << endl;
    }
    cout << endl;

    // export image
    if (!compress && writeHeader (img, imagename, filename) < 0)
    {
        cout << "error: conversion failed" << endl;
    }
    else if (compress && writeHeaderPNG (img, imagename, filename) < 0)
    {
        cout << "error: conversion failed" << endl;
    }
    else
    {
        cout << "conversion succeeded" << endl;
    }

    return 0;
}

//---------------------------------------------------------------------------
