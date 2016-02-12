/* X11 helper functions needed for both GTK and SDL platforms. */

#include <X11/Xlib.h>
#include <GL/glx.h>

namespace {
static const std::map<int, int> base_attrib_map {
	{GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT},
	{GLX_X_RENDERABLE, True},
	{GLX_RED_SIZE, 4},
	{GLX_GREEN_SIZE, 4},
	{GLX_BLUE_SIZE, 4},
	{GLX_DEPTH_SIZE, 8}
};

// Turns an int->int map into an attribute list suitable for any GLX calls.
std::unique_ptr<int[]> MakeGLXAttribList(const std::map<int, int> &map)
{
	// We need two ints for every attribute, plus one as a sentinel
	auto list = std::make_unique<int[]>(map.size() * 2 + 1);
	int *cursor = list.get();
	for(const auto &attrib : map)
	{
		*cursor++ = attrib.first;
		*cursor++ = attrib.second;
	}
	/* *cursor = None; */
	return list;
}

void EnumerateMultiSamples(Display * const dpy, std::vector<int>& samples)
{
	std::map<int, int> attribs = base_attrib_map;
	attribs[GLX_SAMPLE_BUFFERS_ARB] = 1;

	int config_count = 0;
	GLXFBConfig *configs = glXChooseFBConfig(dpy, DefaultScreen(dpy), MakeGLXAttribList(attribs).get(), &config_count);

	std::set<int> multisamples;
	for(int i = 0; i < config_count; ++i)
	{
		int v_samples;
		glXGetFBConfigAttrib(dpy, configs[i], GLX_SAMPLES, &v_samples);
		multisamples.insert(v_samples);
	}

	XFree(configs);
	samples.assign(multisamples.cbegin(), multisamples.cend());
}

// This function picks an acceptable GLXFBConfig. To do this, we first
// request a list of framebuffer configs with no less than 4 bits per color;
// no less than 8 bits of depth buffer; if multisampling is not -1,
// with at least the requested number of samples; and with double buffering.
// If that returns no suitable configs, we retry with only a single buffer.
GLXFBConfig PickGLXFBConfig(Display* dpy, int multisampling)
{
	std::map<int, int> attrib_map = base_attrib_map;

	if (multisampling >= 0)
	{
		attrib_map[GLX_SAMPLE_BUFFERS] = multisampling > 0 ? 1 : 0;
		attrib_map[GLX_SAMPLES] = multisampling;
	}

	GLXFBConfig *configs = NULL;
	int config_count;
	// Find a double-buffered FB config
	attrib_map[GLX_DOUBLEBUFFER] = True;
	std::unique_ptr<int[]> attribs = MakeGLXAttribList(attrib_map);
	configs = glXChooseFBConfig(dpy, DefaultScreen(dpy), attribs.get(), &config_count);
	if (config_count == 0)
	{
		// If none exists, try to find a single-buffered one
		if (configs != NULL)
			XFree(configs);
		attrib_map[GLX_DOUBLEBUFFER] = False;
		attribs = MakeGLXAttribList(attrib_map);
		configs = glXChooseFBConfig(dpy, DefaultScreen(dpy), attribs.get(), &config_count);
	}

	GLXFBConfig config = NULL;
	if (config_count > 0)
	{
		config = configs[0];
	}

	XFree(configs);
	return config;
}
}
