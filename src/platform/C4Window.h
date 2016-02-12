/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2005-2009, RedWolf Design GmbH, http://www.clonk.de/
 * Copyright (c) 2009-2013, The OpenClonk Team and contributors
 *
 * Distributed under the terms of the ISC license; see accompanying file
 * "COPYING" for details.
 *
 * "Clonk" is a registered trademark of Matthes Bender, used with permission.
 * See accompanying file "TRADEMARK" for details.
 *
 * To redistribute this file separately, substitute the full license texts
 * for the above references.
 */

/* A wrapper class to OS dependent event and window interfaces */

#ifndef INC_STDWINDOW
#define INC_STDWINDOW

#include <StdBuf.h>

#if defined(USE_WIN32_WINDOWS) || defined(USE_GTK) || defined(USE_CONSOLE)
#define K_ESCAPE 1
#define K_1 2
#define K_2 3
#define K_3 4
#define K_4 5
#define K_5 6
#define K_6 7
#define K_7 8
#define K_8 9
#define K_9 10
#define K_0 11
#define K_MINUS 12
#define K_EQUAL 13
#define K_BACK 14
#define K_TAB 15
#define K_Q 16
#define K_W 17
#define K_E 18
#define K_R 19
#define K_T 20
#define K_Y 21
#define K_U 22
#define K_I 23
#define K_O 24
#define K_P 25
#define K_LEFT_BRACKET 26
#define K_RIGHT_BRACKET 27
#define K_RETURN 28
#define K_CONTROL_L 29
#define K_A 30
#define K_S 31
#define K_D 32
#define K_F 33
#define K_G 34
#define K_H 35
#define K_J 36
#define K_K 37
#define K_L 38
#define K_SEMICOLON 39
#define K_APOSTROPHE 40
#define K_GRAVE_ACCENT 41
#define K_SHIFT_L 42
#define K_BACKSLASH 43
#define K_Z 44
#define K_X 45
#define K_C 46
#define K_V 47
#define K_B 48
#define K_N 49
#define K_M 50
#define K_COMMA 51
#define K_PERIOD 52
#define K_SLASH 53
#define K_SHIFT_R 54
#define K_MULTIPLY 55
#define K_ALT_L 56 
#define K_SPACE 57
#define K_CAPS 58
#define K_F1 59
#define K_F2 60
#define K_F3 61
#define K_F4 62
#define K_F5 63
#define K_F6 64
#define K_F7 65
#define K_F8 66
#define K_F9 67
#define K_F10 68
#define K_NUM 69
#define K_SCROLL 70
#define K_SUBTRACT 74
#define K_ADD 78
#define K_86 86
#define K_F11 87
#define K_F12 88

/*
// starting from here, scancodes between windows and linux differ
// this is not used because the windows scancodes are converted to
// unix scancodes in C4WindowWin32.cpp ConvertToUnixScancode
#if defined(USE_WIN32_WINDOWS)
#define K_HOME 71
#define K_UP 72
#define K_PAGEUP 73
#define K_LEFT 75
#define K_CENTER 76
#define K_RIGHT 77
#define K_END 79
#define K_DOWN 80
#define K_PAGEDOWN 81
#define K_INSERT 82
#define K_DELETE 83
#define K_WIN_L 91
#define K_WIN_R 92
#define K_MENU 93
#define K_PAUSE 69 // same as numlock?!
#define K_PRINT 55 // same as multiply?!
#define K_ALT_R K_ALT_L // 29 56
#define K_CONTROL_R K_CONTROL_L // 29 29
#define K_NUM_RETURN K_RETURN // 28 57
#define K_NUM7 K_HOME
#define K_NUM8 K_UP
#define K_NUM9 K_PAGEUP
#define K_NUM4 K_LEFT
#define K_NUM5 K_CENTER
#define K_NUM6 K_RIGHT
#define K_NUM1 K_END
#define K_NUM2 K_DOWN
#define K_NUM3 K_PAGEDOWN
#define K_NUM0 K_INSERT
#define K_DECIMAL K_DELETE
#define K_DIVIDE K_SLASH
#elif defined(USE_X11) || defined(USE_CONSOLE)
*/
#define K_NUM7 71
#define K_NUM8 72
#define K_NUM9 73
#define K_NUM4 75
#define K_NUM5 76
#define K_NUM6 77
#define K_NUM1 79
#define K_NUM2 80
#define K_NUM3 81
#define K_NUM0 82
#define K_DECIMAL 83
#define K_DIVIDE 98

#define K_ALT_R 100
#define K_CONTROL_R 97
#define K_NUM_RETURN 96

#define K_HOME 102
#define K_UP 103
#define K_PAGEUP 104
#define K_LEFT 105
#define K_RIGHT 106
#define K_END 107
#define K_DOWN 108
#define K_PAGEDOWN 109
#define K_INSERT 110
#define K_DELETE 111
#define K_WIN_L 125
#define K_WIN_R 126
#define K_MENU 127
#define K_PAUSE 119
#define K_PRINT 99
#define K_CENTER 76

#elif defined(USE_SDL_MAINLOOP)
#include <SDL.h>
#define K_SHIFT_L SDL_SCANCODE_LSHIFT
#define K_SHIFT_R SDL_SCANCODE_RSHIFT
#define K_CONTROL_L SDL_SCANCODE_LCTRL
#define K_CONTROL_R SDL_SCANCODE_RCTRL
#define K_ALT_L SDL_SCANCODE_LALT
#define K_ALT_R SDL_SCANCODE_RALT
#define K_F1 SDL_SCANCODE_F1
#define K_F2 SDL_SCANCODE_F2
#define K_F3 SDL_SCANCODE_F3
#define K_F4 SDL_SCANCODE_F4
#define K_F5 SDL_SCANCODE_F5
#define K_F6 SDL_SCANCODE_F6
#define K_F7 SDL_SCANCODE_F7
#define K_F8 SDL_SCANCODE_F8
#define K_F9 SDL_SCANCODE_F9
#define K_F10 SDL_SCANCODE_F10
#define K_F11 SDL_SCANCODE_F11
#define K_F12 SDL_SCANCODE_F12
#define K_ADD SDL_SCANCODE_KP_PLUS
#define K_SUBTRACT SDL_SCANCODE_KP_MINUS
#define K_MULTIPLY SDL_SCANCODE_KP_MULTIPLY
#define K_ESCAPE SDL_SCANCODE_ESCAPE
#define K_PAUSE SDL_SCANCODE_PAUSE
#define K_TAB SDL_SCANCODE_TAB
#define K_RETURN SDL_SCANCODE_RETURN
#define K_DELETE SDL_SCANCODE_DELETE
#define K_INSERT SDL_SCANCODE_INSERT
#define K_BACK SDL_SCANCODE_BACKSPACE
#define K_SPACE SDL_SCANCODE_SPACE
#define K_UP SDL_SCANCODE_UP
#define K_DOWN SDL_SCANCODE_DOWN
#define K_LEFT SDL_SCANCODE_LEFT
#define K_RIGHT SDL_SCANCODE_RIGHT
#define K_HOME SDL_SCANCODE_HOME
#define K_END SDL_SCANCODE_END
#define K_SCROLL SDL_SCANCODE_SCROLLLOCK
#define K_MENU SDL_SCANCODE_MENU
#define K_PAGEUP SDL_SCANCODE_PAGEUP
#define K_PAGEDOWN SDL_SCANCODE_PAGEDOWN
#define K_1 SDL_SCANCODE_1
#define K_2 SDL_SCANCODE_2
#define K_3 SDL_SCANCODE_3
#define K_4 SDL_SCANCODE_4
#define K_5 SDL_SCANCODE_5
#define K_6 SDL_SCANCODE_6
#define K_7 SDL_SCANCODE_7
#define K_8 SDL_SCANCODE_8
#define K_9 SDL_SCANCODE_9
#define K_0 SDL_SCANCODE_A
#define K_A SDL_SCANCODE_A
#define K_B SDL_SCANCODE_B
#define K_C SDL_SCANCODE_C
#define K_D SDL_SCANCODE_D
#define K_E SDL_SCANCODE_E
#define K_F SDL_SCANCODE_F
#define K_G SDL_SCANCODE_G
#define K_H SDL_SCANCODE_H
#define K_I SDL_SCANCODE_I
#define K_J SDL_SCANCODE_J
#define K_K SDL_SCANCODE_K
#define K_L SDL_SCANCODE_L
#define K_M SDL_SCANCODE_M
#define K_N SDL_SCANCODE_N
#define K_O SDL_SCANCODE_O
#define K_P SDL_SCANCODE_P
#define K_Q SDL_SCANCODE_Q
#define K_R SDL_SCANCODE_R
#define K_S SDL_SCANCODE_S
#define K_T SDL_SCANCODE_T
#define K_U SDL_SCANCODE_U
#define K_V SDL_SCANCODE_V
#define K_W SDL_SCANCODE_W
#define K_X SDL_SCANCODE_X
#define K_Y SDL_SCANCODE_Y
#define K_Z SDL_SCANCODE_Z
#define K_MINUS SDL_SCANCODE_MINUS
#define K_EQUAL SDL_SCANCODE_EQUALS
#define K_LEFT_BRACKET SDL_SCANCODE_LEFTBRACKET
#define K_RIGHT_BRACKET SDL_SCANCODE_RIGHTBRACKET
#define K_SEMICOLON SDL_SCANCODE_SEMICOLON
#define K_APOSTROPHE SDL_SCANCODE_APOSTROPHE
#define K_GRAVE_ACCENT SDL_SCANCODE_GRAVE
#define K_BACKSLASH SDL_SCANCODE_BACKSLASH
#define K_COMMA SDL_SCANCODE_COMMA
#define K_PERIOD SDL_SCANCODE_PERIOD
#define K_SLASH SDL_SCANCODE_SLASH
#define K_CAPS SDL_SCANCODE_CAPSLOCK
#define K_NUM SDL_SCANCODE_NUMLOCKCLEAR
#define K_NUM7 SDL_SCANCODE_KP_7
#define K_NUM8 SDL_SCANCODE_KP_8
#define K_NUM9 SDL_SCANCODE_KP_9
#define K_NUM4 SDL_SCANCODE_KP_4
#define K_NUM5 SDL_SCANCODE_KP_5
#define K_NUM6 SDL_SCANCODE_KP_6
#define K_NUM1 SDL_SCANCODE_KP_1
#define K_NUM2 SDL_SCANCODE_KP_2
#define K_NUM3 SDL_SCANCODE_KP_3
#define K_NUM0 SDL_SCANCODE_KP_0
#define K_DECIMAL SDL_SCANCODE_KP_PERIOD
#define K_86 SDL_SCANCODE_NONUSBACKSLASH
#define K_NUM_RETURN SDL_SCANCODE_KP_ENTER
#define K_DIVIDE SDL_SCANCODE_KP_DIVIDE
#define K_WIN_L SDL_SCANCODE_LGUI
#define K_WIN_R SDL_SCANCODE_RGUI
#define K_PRINT SDL_SCANCODE_PRINTSCREEN
#elif defined(USE_COCOA)
#import "ObjectiveCAssociated.h"
// FIXME
// declare as extern variables and initialize them in StdMacWindow.mm so as to not include objc headers
const int CocoaKeycodeOffset = 300;
extern C4KeyCode K_SHIFT_L;
extern C4KeyCode K_SHIFT_R;
extern C4KeyCode K_CONTROL_L;
extern C4KeyCode K_CONTROL_R;
extern C4KeyCode K_ALT_L;
extern C4KeyCode K_ALT_R;
extern C4KeyCode K_COMMAND_L;
extern C4KeyCode K_COMMAND_R;
extern C4KeyCode K_F1;
extern C4KeyCode K_F2;
extern C4KeyCode K_F3;
extern C4KeyCode K_F4;
extern C4KeyCode K_F5;
extern C4KeyCode K_F6;
extern C4KeyCode K_F7;
extern C4KeyCode K_F8;
extern C4KeyCode K_F9;
extern C4KeyCode K_F10;
extern C4KeyCode K_F11;
extern C4KeyCode K_F12;
extern C4KeyCode K_ADD;
extern C4KeyCode K_SUBTRACT;
extern C4KeyCode K_MULTIPLY;
extern C4KeyCode K_ESCAPE;
extern C4KeyCode K_PAUSE;
extern C4KeyCode K_TAB;
extern C4KeyCode K_RETURN;
extern C4KeyCode K_DELETE;
extern C4KeyCode K_INSERT;
extern C4KeyCode K_BACK;
extern C4KeyCode K_SPACE;
extern C4KeyCode K_UP;
extern C4KeyCode K_DOWN;
extern C4KeyCode K_LEFT;
extern C4KeyCode K_RIGHT;
extern C4KeyCode K_HOME;
extern C4KeyCode K_END;
extern C4KeyCode K_SCROLL;
extern C4KeyCode K_MENU;
extern C4KeyCode K_PAGEUP;
extern C4KeyCode K_PAGEDOWN;
extern C4KeyCode K_M;
extern C4KeyCode K_T;
extern C4KeyCode K_W;
extern C4KeyCode K_I;
extern C4KeyCode K_C;
extern C4KeyCode K_V;
extern C4KeyCode K_X;
extern C4KeyCode K_A;
#endif

#ifdef USE_GTK
// Forward declaration because xlib.h is evil
typedef struct __GLXFBConfigRec *GLXFBConfig;
#endif

class C4Window
#ifdef USE_COCOA
	: public ObjectiveCAssociated
#endif
{
public:
	enum WindowKind
	{
		W_GuiWindow,
		W_Console,
		W_Viewport,
		W_Fullscreen,
		W_Control // wrapper to a render target control inside a window
	};
public:
	C4Window ();
	virtual ~C4Window ();
	bool Active;
	C4Surface * pSurface;
	WindowKind eKind;
	virtual void Clear();
	// Only when the wm requests a close
	// For example, when the user clicks the little x in the corner or uses Alt-F4
	virtual void Close() = 0;
	// Keypress(es) translated to a char
	virtual void CharIn(const char *) { }

	// Reinitialize the window with updated configuration settings.
	// Keep window kind, title and size as they are. Currently the only point
	// at which it makes sense for this function to be called is when the
	// multisampling configuration option changes, since, for the change to
	// take effect, we need to choose another visual or pixel format, respectively.
	virtual bool ReInit(C4AbstractApp* pApp);

	// Creates a list of available samples for multisampling
	virtual void EnumerateMultiSamples(std::vector<int>& samples) const;

	bool StorePosition(const char *szWindowName, const char *szSubKey, bool fStoreSize = true);
	bool RestorePosition(const char *szWindowName, const char *szSubKey, bool fHidden = false);
	bool GetSize(C4Rect * pRect);
	void SetSize(unsigned int cx, unsigned int cy); // resize
	void SetTitle(const char * Title);
	void FlashWindow();
	void GrabMouse(bool grab);
	// request that this window be redrawn in the near future (including immediately)
	virtual void RequestUpdate();
	// Invokes actual drawing code - should not be called directly
	virtual void PerformUpdate();

public:
#if defined(USE_WIN32_WINDOWS)
	HWND hWindow;
	virtual bool Win32DialogMessageHandling(MSG * msg) { return false; };
#elif defined(USE_GTK)
	/*GtkWidget*/void * window;
	// Set by Init to the widget which is used as a
	// render target, which can be the whole window.
	/*GtkWidget*/void * render_widget;
#elif defined(USE_SDL_MAINLOOP)
	SDL_Window * window;
#endif
#ifdef USE_WGL
	HWND renderwnd;
#elif defined(USE_GTK)
	unsigned long renderwnd;
#endif
protected:
#if defined(USE_GTK)
	bool FindFBConfig(int samples, GLXFBConfig *info);

	// The GLXFBConfig the window was created with
	GLXFBConfig Info;
	unsigned long handlerDestroy;
#endif
	virtual C4Window * Init(WindowKind windowKind, C4AbstractApp * pApp, const char * Title, const C4Rect * size);
	friend class C4Draw;
	friend class CStdGL;
	friend class CStdGLCtx;
	friend class C4AbstractApp;
};

#endif // INC_STDWINDOW
