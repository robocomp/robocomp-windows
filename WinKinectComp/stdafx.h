//------------------------------------------------------------------------------
// <copyright file="stdafx.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

// include file for standard system and project includes

#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
#endif

// Windows Header Files
#include <windows.h>

#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>

// Kinect Header files
#include <Kinect.h>
#include <Kinect.Face.h>

#define _CRT_SECURE_CPP_OVERLOAD_SECURE_NAMES 1
#include <new>
#include <strsafe.h>
#include <string>

#pragma comment (lib, "d2d1.lib")

#ifdef _UNICODE
#if defined _M_IX86
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='x86' publicKeyToken='6595b64144ccf1df' language='*'\"")
#elif defined _M_X64
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='amd64' publicKeyToken='6595b64144ccf1df' language='*'\"")
#else
#pragma comment(linker,"/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")
#endif
#endif



// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

// Case-insensitive wide-character char traits
struct i_char_traits : public std::char_traits<wchar_t>
{
    static bool eq(wchar_t c1, wchar_t c2) { return towupper(c1) == towupper(c2); } 
    static bool ne(wchar_t c1, wchar_t c2) { return towupper(c1) != towupper(c2); } 
    static bool lt(wchar_t c1, wchar_t c2) { return towupper(c1) <  towupper(c2); }
    static int compare(const wchar_t* s1, const wchar_t* s2, size_t n)
    { 
        while( 0 != n-- )
        { 
            if( towupper(*s1) < towupper(*s2) ) return -1; 
            if( towupper(*s1) > towupper(*s2) ) return 1; 
            ++s1;
            ++s2; 
        }

        return 0; 
    }

    static const wchar_t* find(const wchar_t* s, size_t n, wchar_t a)
    {
        int remaining = static_cast<int>(n);
        while( (--remaining >= 0) && (towupper(*s) != towupper(a)) )
        { 
            ++s; 
        } 
        return (remaining >= 0) ? s : NULL; 
    } 
};

typedef std::basic_string<wchar_t, i_char_traits> wistring;