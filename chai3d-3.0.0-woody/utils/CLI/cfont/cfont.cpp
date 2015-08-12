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
    \version   3.0.0 $Rev: 1246 $
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
int writeHeader(cFont &font, string fontname, string filename)
{
    unsigned int   csbase       = font.m_charset.m_base;
    unsigned int   cslineHeight = font.m_charset.m_lineHeight;
    unsigned int   cswidth      = font.m_charset.m_width;
    unsigned int   csheight     = font.m_charset.m_height;
    unsigned int   cspages      = font.m_charset.m_pages;
    unsigned int   imwidth      = font.m_texture->m_image->getWidth();
    unsigned int   imheight     = font.m_texture->m_image->getHeight();
    unsigned int   imbpp        = font.m_texture->m_image->getBytesPerPixel();
    unsigned int   imformat     = font.m_texture->m_image->getFormat();
    int            imbcount     = imwidth*imheight*imbpp;
    int            imptr        = 0;
    unsigned char *imdata       = font.m_texture->m_image->getData();

    // integrity check
    if (cswidth != imwidth || csheight != imheight)
    {
        cout << "error: charset size and image size differ" << endl;
        return -1;
    }

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing \"" << fontname << "\"." << endl;
    out << endl;
    out << "    Automatically generated using CHAI3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert imagename to root name
    fontname = fontname.substr(fontname.find_last_of('/')+1, fontname.length());
    transform(fontname.begin(), fontname.end(),fontname.begin(), ::toupper);
    replace(fontname.begin(), fontname.end(), '.', '_');
    replace(fontname.begin(), fontname.end(), ' ', '_');
    replace(fontname.begin(), fontname.end(), '\'', '_');
    replace(fontname.begin(), fontname.end(), '-', '_');

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
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#ifndef " << filename << "H" << endl;
    out << "#define " << filename << "H" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // include headers (required by convenience functions)
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#include \"graphics/CImage.h\"" << endl;
    out << "#include \"graphics/CFont.h\"" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    // namespace
    out << "namespace chai3d {" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // print joint info
    out << "const unsigned int " << filename << "_WIDTH              = " << cswidth      << ";" << endl;
    out << "const unsigned int " << filename << "_HEIGHT             = " << csheight     << ";" << endl;

    // print font info
    out << "const unsigned int " << filename << "_CHARSET_BASE       = " << csbase       << ";" << endl;
    out << "const unsigned int " << filename << "_CHARSET_LINEHEIGHT = " << cslineHeight << ";" << endl;
    out << "const unsigned int " << filename << "_CHARSET_PAGES      = " << cspages      << ";" << endl;

    // print image info
    out << "const unsigned int " << filename << "_IMAGE_SIZE         = " << imbcount << ";" << endl;
    out << "const unsigned int " << filename << "_IMAGE_BPP          = " << imbpp    << ";" << endl;
    out << "const unsigned int " << filename << "_IMAGE_FORMAT       = " << imformat << ";" << "    // ";
    switch (imformat)
    {
        case GL_RGB:  out << "GL_RGB";  break;
        case GL_RGBA: out << "GL_RGBA"; break;
    }
    out << endl << endl;

    // print charset payload
    out << "const int " << filename << "_CHARSET[] =" << endl;
    out << "{" << endl;
    out << "//  "
            << setw(10) << "x,"
            << setw(10) << "y,"
            << setw(10) << "width,"
            << setw(10) << "height,"
            << setw(10) << "xOff,"
            << setw(10) << "yOff,"
            << setw(10) << "xAdvance,"
            << setw(10) << "page,"
            << endl;
    for (unsigned int c=0; c<256; c++)
    {
        out << "    "
            << setw(9) << font.m_charset.m_chars[c].m_x << ","
            << setw(9) << font.m_charset.m_chars[c].m_y << ","
            << setw(9) << font.m_charset.m_chars[c].m_width << ","
            << setw(9) << font.m_charset.m_chars[c].m_height << ","
            << setw(9) << font.m_charset.m_chars[c].m_xOffset << ","
            << setw(9) << font.m_charset.m_chars[c].m_yOffset << ","
            << setw(9) << font.m_charset.m_chars[c].m_xAdvance << ","
            << setw(9) << font.m_charset.m_chars[c].m_page;
        if (c < 256) out << ",";
        if (c > 31 && c < 127) out << "    // [" << setw(3) << c << ": '" << (char)c << "']";
        else                   out << "    // [" << setw(3) << c << "]";
        out << endl;
    }
    out << endl << "};" << endl << endl;

    // print image payload
    out << "const unsigned char " << filename << "_BYTEARRAY[] =" << endl;
    out << "{";
    for (unsigned int j=0; j<imheight; j++)
    {
        out << endl << "    ";
        for (unsigned int i=0; i<imwidth; i++)
        {
            for (unsigned int h=0; h<imbpp; h++)
            {
                out << "0x" << hex << (int)(imdata[imptr++]);
                if (imptr < imbcount) out << ", ";
            }
        }
    }
    out << endl << "};" << endl << endl << endl;

    // convenience function: allocate image
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;
    out << endl;
    out << "inline cFont *NEW_" << filename << "()" << endl;
    out << "{" << endl;
    out << "    cImage        *img       = new cImage(" << filename << "_WIDTH, " << filename << "_HEIGHT, " << filename << "_IMAGE_FORMAT);" << endl;
    out << "    unsigned char *bytearray = new unsigned char[" << filename << "_IMAGE_SIZE];" << endl;
    out << endl;
    out << "    memcpy(bytearray, " << filename << "_BYTEARRAY, " << filename << "_IMAGE_SIZE);" << endl;
    out << "    img->setData(bytearray, " << filename << "_IMAGE_SIZE, true);" << endl;
    out << endl;
    out << "    cFont *font = new cFont();" << endl;
    out << "    font->m_charset.m_base       = " << filename << "_CHARSET_BASE;" << endl;
    out << "    font->m_charset.m_lineHeight = " << filename << "_CHARSET_LINEHEIGHT;" << endl;
    out << "    font->m_charset.m_width      = " << filename << "_WIDTH;" << endl;
    out << "    font->m_charset.m_height     = " << filename << "_HEIGHT;" << endl;
    out << "    font->m_charset.m_pages      = " << filename << "_CHARSET_PAGES;" << endl;
    out << "    for (int i=0; i<256; i++)" << endl;
    out << "    {" << endl;
    out << "        font->m_charset.m_chars[i].m_x        = " << filename << "_CHARSET[8*i  ];" << endl;
    out << "        font->m_charset.m_chars[i].m_y        = " << filename << "_CHARSET[8*i+1];" << endl;
    out << "        font->m_charset.m_chars[i].m_width    = " << filename << "_CHARSET[8*i+2];" << endl;
    out << "        font->m_charset.m_chars[i].m_height   = " << filename << "_CHARSET[8*i+3];" << endl;
    out << "        font->m_charset.m_chars[i].m_xOffset  = " << filename << "_CHARSET[8*i+4];" << endl;
    out << "        font->m_charset.m_chars[i].m_yOffset  = " << filename << "_CHARSET[8*i+5];" << endl;
    out << "        font->m_charset.m_chars[i].m_xAdvance = " << filename << "_CHARSET[8*i+6];" << endl;
    out << "        font->m_charset.m_chars[i].m_page     = " << filename << "_CHARSET[8*i+7];" << endl;
    out << "    }" << endl;
    out << "    font->m_charset.preProcess();" << endl;
    out << endl;
    out << "    font->m_texture->setImage(std::shared_ptr<cImage>(img));" << endl;
    out << endl;
    out << "    return (font);" << endl;
    out << "}" << endl << endl << endl;

    // close namespace
    out << "//---------------------------------------------------------------------------" << endl;
    out << "} // namespace chai3d" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // tidy up
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#endif" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    out.close();

    return 0;
}


// PNG header file printer
int writeHeaderPNG(cFont &font, string fontname, string  filename)
{
    const int lineWidth = 256;

    unsigned int   csbase       = font.m_charset.m_base;
    unsigned int   cslineHeight = font.m_charset.m_lineHeight;
    unsigned int   cswidth      = font.m_charset.m_width;
    unsigned int   csheight     = font.m_charset.m_height;
    unsigned int   cspages      = font.m_charset.m_pages;
    unsigned int   imptr        = 0;

    // integrity check
    if (cswidth != font.m_texture->m_image->getWidth() || csheight != font.m_texture->m_image->getHeight())
    {
        cout << "error: charset size and image size differ" << endl;
        return -1;
    }

    ofstream out(filename.c_str());

    out << "//===========================================================================" << endl;
    out << "/*" << endl;
    out << "    Header file containing \"" << fontname << "\"." << endl;
    out << endl;
    out << "    Automatically generated using CHAI3D visualization and haptics library." << endl;
    out << "    http://www.chai3d.org" << endl;
    out << "*/" << endl;
    out << "//===========================================================================" << endl;

    // convert imagename to root name
    fontname = fontname.substr(fontname.find_last_of('/')+1, fontname.length());
    transform(fontname.begin(), fontname.end(),fontname.begin(), ::toupper);
    replace(fontname.begin(), fontname.end(), '.', '_');
    replace(fontname.begin(), fontname.end(), ' ', '_');
    replace(fontname.begin(), fontname.end(), '\'', '_');
    replace(fontname.begin(), fontname.end(), '-', '_');

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
    if (!cSavePNG(font.m_texture->m_image.get(), &buffer, &len))
    {
        cout << "PNG compression failed" << endl;
        return -1;
    }

    // avoid conflicts
    out << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#ifndef " << filename << "H" << endl;
    out << "#define " << filename << "H" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // include headers (required by convenience functions)
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#include \"graphics/CImage.h\"" << endl;
    out << "#include \"graphics/CFont.h\"" << endl;
    out << "#include \"files/CFileImagePNG.h\"" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // namespace
    out << "//---------------------------------------------------------------------------" << endl;
    out << "namespace chai3d {" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // print joint info
    out << "const unsigned int " << filename << "_WIDTH              = " << cswidth      << ";" << endl;
    out << "const unsigned int " << filename << "_HEIGHT             = " << csheight     << ";" << endl;

    // print font info
    out << "const unsigned int " << filename << "_CHARSET_BASE       = " << csbase       << ";" << endl;
    out << "const unsigned int " << filename << "_CHARSET_LINEHEIGHT = " << cslineHeight << ";" << endl;
    out << "const unsigned int " << filename << "_CHARSET_PAGES      = " << cspages      << ";" << endl;
    out << endl;

    // print charset payload
    out << "const int " << filename << "_CHARSET[] =" << endl;
    out << "{" << endl;
    out << "//  "
            << setw(10) << "x,"
            << setw(10) << "y,"
            << setw(10) << "width,"
            << setw(10) << "height,"
            << setw(10) << "xOff,"
            << setw(10) << "yOff,"
            << setw(10) << "xAdvance,"
            << setw(10) << "page,"
            << endl;
    for (unsigned int c=0; c<256; c++)
    {
        out << "    "
            << setw(9) << font.m_charset.m_chars[c].m_x << ","
            << setw(9) << font.m_charset.m_chars[c].m_y << ","
            << setw(9) << font.m_charset.m_chars[c].m_width << ","
            << setw(9) << font.m_charset.m_chars[c].m_height << ","
            << setw(9) << font.m_charset.m_chars[c].m_xOffset << ","
            << setw(9) << font.m_charset.m_chars[c].m_yOffset << ","
            << setw(9) << font.m_charset.m_chars[c].m_xAdvance << ","
            << setw(9) << font.m_charset.m_chars[c].m_page;
        if (c < 256) out << ",";
        if (c > 31 && c < 127) out << "    // [" << setw(3) << c << ": '" << (char)c << "']";
        else                   out << "    // [" << setw(3) << c << "]";
        out << endl;
    }
    out << endl << "};" << endl << endl;

    // print image payload
    out << "const unsigned char " << filename << "_BYTEARRAY[] =" << endl;
    out << "{";
    for (unsigned int s=0; s<len; s++)
    {
        if (s%lineWidth == 0) out << endl << "\t";
        out << "0x" << hex << setw(2) << setfill('0') << (int)(((unsigned char*)buffer)[imptr++]);
        if (imptr < len) out << ", ";
    }
    out << endl << "};" << endl << endl << endl;

    // convenience function: allocate image
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;
    out << endl;
    out << "inline cFont *NEW_" << filename << "()" << endl;
    out << "{" << endl;
    out << "    cImage *img = new cImage();" << endl;
    out << "    cLoadPNG(img, " << filename << "_BYTEARRAY, sizeof(" << filename << "_BYTEARRAY));" << endl;
    out << endl;
    out << "    cFont *font = new cFont();" << endl;
    out << "    font->m_charset.m_base       = " << filename << "_CHARSET_BASE;" << endl;
    out << "    font->m_charset.m_lineHeight = " << filename << "_CHARSET_LINEHEIGHT;" << endl;
    out << "    font->m_charset.m_width      = " << filename << "_WIDTH;" << endl;
    out << "    font->m_charset.m_height     = " << filename << "_HEIGHT;" << endl;
    out << "    font->m_charset.m_pages      = " << filename << "_CHARSET_PAGES;" << endl;
    out << "    for (int i=0; i<256; i++)" << endl;
    out << "    {" << endl;
    out << "        font->m_charset.m_chars[i].m_x        = " << filename << "_CHARSET[8*i  ];" << endl;
    out << "        font->m_charset.m_chars[i].m_y        = " << filename << "_CHARSET[8*i+1];" << endl;
    out << "        font->m_charset.m_chars[i].m_width    = " << filename << "_CHARSET[8*i+2];" << endl;
    out << "        font->m_charset.m_chars[i].m_height   = " << filename << "_CHARSET[8*i+3];" << endl;
    out << "        font->m_charset.m_chars[i].m_xOffset  = " << filename << "_CHARSET[8*i+4];" << endl;
    out << "        font->m_charset.m_chars[i].m_yOffset  = " << filename << "_CHARSET[8*i+5];" << endl;
    out << "        font->m_charset.m_chars[i].m_xAdvance = " << filename << "_CHARSET[8*i+6];" << endl;
    out << "        font->m_charset.m_chars[i].m_page     = " << filename << "_CHARSET[8*i+7];" << endl;
    out << "    }" << endl;
    out << "    font->m_charset.preProcess();" << endl;
    out << endl;
    out << "    font->m_texture->setImage(std::shared_ptr<cImage>(img));" << endl;
    out << endl;
    out << "    return (font);" << endl;
    out << "}" << endl << endl << endl;

    // close namespace
    out << "//---------------------------------------------------------------------------" << endl;
    out << "} // namespace chai3d" << endl;
    out << "//---------------------------------------------------------------------------" << endl;
    out << endl;

    // tidy up
    out << "//---------------------------------------------------------------------------" << endl;
    out << "#endif" << endl;
    out << "//---------------------------------------------------------------------------" << endl;

    out.close();

    return 0;
}


// simple usage printer
int usage()
{
    cout << endl << "04-cfont [-c] font.fnt [-o header.h]" << endl;
    cout << "\t-c\tuse PNG compression to store image in header" << endl;
    cout << "\t-o\tspecify header filename" << endl;
    cout << "\t-h\tdisplay this message" << endl << endl;

    return -1;
}


//===========================================================================
/*
    DEMO:    04-cfont.cpp

    This example takes font information from a .fnt file. It reads the
    corresponding image (that can be in any CHAI3D supported format) and
    produces a C/C++ compatible header containing the font information.
    This allows programmers to easily embed fonts into their executables.
 */
//===========================================================================

int main(int argc, char* argv[])
{
    cImage img;
    cFont  font;
    string fontname;
    string filename;
    bool   compressed = false;

    // process arguments
    if (argc < 2) return usage();
    for (int i=1; i<argc; i++)
    {
        if (argv[i][0] != '-') {
            if (fontname.length() > 0) return usage();
            fontname = string(argv[i]);
            filename  = fontname;
            unsigned int index = fontname.find_last_of('.');
            if (index < fontname.length())
            {
                filename.replace (index, 4, ".h");
            }
            else
            {
                cout << "error: incorrect font file name" << endl;
                return -1;
            }
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
                compressed = true;
                break;
            default:
                return usage ();
        }
    }

    // pretty message
    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Font Converter" << endl;
    cout << "Copyright 2003-2014" << endl;
    cout << "-----------------------------------" << endl;
    cout << endl;

    // report action
    cout << "converting " << fontname << " to " << filename << "..." << endl;

    // read font
    if (!font.loadFromFile (fontname))
    {
        cout << "error: cannot load font file " << fontname << endl;
        return -1;
    }
    else 
    {
        cout << "font load succeeded" << endl;
    }
    cout << endl;

    // export image
    if (!compressed && writeHeader (font, fontname, filename) < 0)
    {
        cout << "error: conversion failed" << endl;
        return -1;
    }
    else if (compressed && writeHeaderPNG (font, fontname, filename) < 0)
    {
        cout << "error: conversion failed" << endl;
        return -1;
    }
    else
    {
        cout << "conversion succeeded" << endl;
    }

    return 0;
}

//---------------------------------------------------------------------------
