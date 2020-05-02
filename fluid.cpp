#include <math.h>
#include <iostream>

#include <stdlib.h>
#include <sstream> 
#include <fstream>
#ifdef _OGLES3
#include "OpenGLES/FrmGLES3.h"
//#include <GLES3/gl3.h>
//#include <GLES2/gl2ext.h>
#else
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3.h>
#endif

#include <glm/glm.hpp>
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_transform.hpp"
//#include "glm/detail/func_geometric.hpp"
#include "glm/geometric.hpp"

using namespace std;

extern ofstream g_logfile;
#include "fluid.h"






#ifndef GL_ES
#define GL_ES
#endif


#ifdef GL_ES
#define STRINGIZE(shader)	"#version 300 es\n" "#pragma debug(on)\n" "precision mediump float;\n" #shader
//#define STRINGIZE(shader)	"#version 300 es\n" #shader
#else
#define STRINGIZE(shader)	"#version 330 core\n" #shader
#endif

//{{ Vertex Shaders
const GLchar *vs_fluid = STRINGIZE(
	in vec4 Position;

	void main()
	{
		gl_Position = Position;
	}
);
//}} Vertex Shaders


//{{ Fragment Shaders
const GLchar * fs_fill = STRINGIZE(
	out vec3 FragColor;

	void main()
	{
		FragColor = vec3(1.0f, 0.0f, 0.0f);
	}
);


const GLchar * fs_advect = STRINGIZE(
	out vec4 FragColor;

	uniform sampler2D VelocityTexture;
	uniform sampler2D SourceTexture;
	uniform sampler2D Obstacles;

	uniform vec2 InverseSize;
	uniform float TimeStep;
	uniform float Dissipation;

	void main()
	{
		vec2 fragCoord = gl_FragCoord.xy;
		float solid = texture(Obstacles, InverseSize * fragCoord).x;

		if (solid > 0.0f)
		{
			FragColor = vec4(0.0f);
			return;
		}

		vec2 u = texture(VelocityTexture, InverseSize * fragCoord).xy;
		vec2 coord = InverseSize * (fragCoord - TimeStep * u);
		FragColor = Dissipation * texture(SourceTexture, coord);
	}
);


const GLchar * fs_jacobi = STRINGIZE(
	out vec4 FragColor;

	uniform sampler2D Pressure;
	uniform sampler2D Divergence;
	uniform sampler2D Obstacles;

	uniform float Alpha;
	uniform float InverseBeta;

	void main()
	{
		ivec2 T = ivec2(gl_FragCoord.xy);

		// Find neighboring pressure:
		vec4 pN = texelFetchOffset(Pressure, T, 0, ivec2(0, 1));
		vec4 pS = texelFetchOffset(Pressure, T, 0, ivec2(0, -1));
		vec4 pE = texelFetchOffset(Pressure, T, 0, ivec2(1, 0));
		vec4 pW = texelFetchOffset(Pressure, T, 0, ivec2(-1, 0));
		vec4 pC = texelFetch(Pressure, T, 0);

		// Find neighboring obstacles:
		vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
		vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
		vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
		vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

		// Use center pressure for solid cells:
		if (oN.x > 0.0f) pN = pC;
		if (oS.x > 0.0f) pS = pC;
		if (oE.x > 0.0f) pE = pC;
		if (oW.x > 0.0f) pW = pC;

		vec4 bC = texelFetch(Divergence, T, 0);
		FragColor = (pW + pE + pS + pN + Alpha * bC) * InverseBeta;
	}
);


const GLchar * fs_subtract_gradient = STRINGIZE(
	out vec2 FragColor;

	uniform sampler2D Velocity;
	uniform sampler2D Pressure;
	uniform sampler2D Obstacles;
	uniform float GradientScale;

	void main()
	{
		ivec2 T = ivec2(gl_FragCoord.xy);

		vec3 oC = texelFetch(Obstacles, T, 0).xyz;
		if (oC.x > 0.0f)
		{
			FragColor = oC.yz;
			return;
		}

		// Find neighboring pressure:
		float pN = texelFetchOffset(Pressure, T, 0, ivec2(0, 1)).r;
		float pS = texelFetchOffset(Pressure, T, 0, ivec2(0, -1)).r;
		float pE = texelFetchOffset(Pressure, T, 0, ivec2(1, 0)).r;
		float pW = texelFetchOffset(Pressure, T, 0, ivec2(-1, 0)).r;
		float pC = texelFetch(Pressure, T, 0).r;

		// Find neighboring obstacles:
		vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
		vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
		vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
		vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

		// Use center pressure for solid cells:
		vec2 obstV = vec2(0);
		vec2 vMask = vec2(1);

		if (oN.x > 0.0f) { pN = pC; obstV.y = oN.z; vMask.y = 0.0f; }
		if (oS.x > 0.0f) { pS = pC; obstV.y = oS.z; vMask.y = 0.0f; }
		if (oE.x > 0.0f) { pE = pC; obstV.x = oE.y; vMask.x = 0.0f; }
		if (oW.x > 0.0f) { pW = pC; obstV.x = oW.y; vMask.x = 0.0f; }

		// Enforce the free-slip boundary condition:
		vec2 oldV = texelFetch(Velocity, T, 0).xy;
		vec2 grad = vec2(pE - pW, pN - pS) * GradientScale;
		vec2 newV = oldV - grad;
		FragColor = (vMask * newV) + obstV;
	}
);


const GLchar * fs_compute_divergence = STRINGIZE(
	out float FragColor;

	uniform sampler2D Velocity;
	uniform sampler2D Obstacles;
	uniform float HalfInverseCellSize;

	void main()
	{
		ivec2 T = ivec2(gl_FragCoord.xy);

		// Find neighboring velocities:
		vec2 vN = texelFetchOffset(Velocity, T, 0, ivec2(0, 1)).xy;
		vec2 vS = texelFetchOffset(Velocity, T, 0, ivec2(0, -1)).xy;
		vec2 vE = texelFetchOffset(Velocity, T, 0, ivec2(1, 0)).xy;
		vec2 vW = texelFetchOffset(Velocity, T, 0, ivec2(-1, 0)).xy;

		// Find neighboring obstacles:
		vec3 oN = texelFetchOffset(Obstacles, T, 0, ivec2(0, 1)).xyz;
		vec3 oS = texelFetchOffset(Obstacles, T, 0, ivec2(0, -1)).xyz;
		vec3 oE = texelFetchOffset(Obstacles, T, 0, ivec2(1, 0)).xyz;
		vec3 oW = texelFetchOffset(Obstacles, T, 0, ivec2(-1, 0)).xyz;

		// Use obstacle velocities for solid cells:
		if (oN.x > 0.0f) vN = oN.yz;
		if (oS.x > 0.0f) vS = oS.yz;
		if (oE.x > 0.0f) vE = oE.yz;
		if (oW.x > 0.0f) vW = oW.yz;

		FragColor = HalfInverseCellSize * (vE.x - vW.x + vN.y - vS.y);
	}
);


const GLchar * fs_splat = STRINGIZE(
	out vec4 FragColor;

	uniform vec2 Point;
	uniform vec3 FillColor;
	uniform float Radius;

	void main()
	{
		float d = distance(Point, gl_FragCoord.xy);
		if (d < Radius)
		{
			float a = (Radius - d) * 0.5f;

			a = min(a, 1.0f);
			FragColor = vec4(FillColor, a);
		}
		else
		{
			FragColor = vec4(0.0f);
		}
	}
);


const GLchar * fs_buoyancy = STRINGIZE(
	out vec2 FragColor;

	uniform sampler2D Velocity;
	uniform sampler2D Temperature;
	uniform sampler2D Density;
	uniform float AmbientTemperature;
	uniform float TimeStep;
	uniform float Sigma;
	uniform float Kappa;

	void main()
	{
		ivec2 TC = ivec2(gl_FragCoord.xy);
		float T = texelFetch(Temperature, TC, 0).r;
		vec2 V = texelFetch(Velocity, TC, 0).xy;

		FragColor = V;

		if (T > AmbientTemperature)
		{
			float D = texelFetch(Density, TC, 0).x;
			FragColor += (TimeStep * (T - AmbientTemperature) * Sigma - D * Kappa ) * vec2(0.0f, 1.0f);
		}
	}
);


const GLchar * fs_visualize = STRINGIZE(
	out vec4 FragColor;

	uniform sampler2D Sampler;
	uniform vec3 FillColor;
	uniform vec2 Scale;

	uniform int		EnableOffset;
	uniform vec2	Offset;
	uniform vec2	Viewport;

	float rand(vec2 c)
	{
		return fract(sin(dot(c.xy, vec2(11.9898, 78.222))) * 33758.55);
	}

	void main()
	{
		vec2 coord = gl_FragCoord.xy;

		if (EnableOffset == 1)
		{
			coord -= Offset;
		}

		float L = texture(Sampler, coord * Scale).r;
		float bias;
		float threshold = Viewport.x / 2.0f;


		bias = Viewport.x / 30.0f;
		bias = 0.1f;
		if (coord.x < threshold - bias)
			FragColor = vec4(FillColor, L);
		else if (coord.x > threshold + bias)
			FragColor = vec4(FillColor.b, FillColor.r, FillColor.g, L);
		else
			FragColor = vec4(FillColor.g, FillColor.b, FillColor.r, L);
	}
);




static int ViewportWidth	= 960;
static int ViewportHeight	= 900;

static int ScreenWidth	= 600;
static int ScreenHeight	= 800;

static int TextureWidth()		{ return (ViewportWidth / 2); }
static int TextureHeight()		{ return (ViewportHeight / 2); }

static int SplatRadius()	{ return ((float) TextureWidth() / 8.0f); }

#define CellSize (1.25f)

static const float	AmbientTemperature = 0.0f;
static const float	ImpulseTemperature = 10.0f;
static const float	ImpulseDensity = 1.0f;
static const int	NumJacobiIterations = 40;
static const float	TimeStep = 0.1f;
static const float	SmokeBuoyancy = 1.5f;
static const float	SmokeWeight = 0.5f;
static const float	GradientScale = 1.125f / CellSize;
static const float	TemperatureDissipation = 0.99f;
static const float	VelocityDissipation = 0.99f;
static const float	DensityDissipation = 0.9999f;
static const int	PositionSlot = 0;



struct ProgramsRec {
    GLuint Advect;
    GLuint Jacobi;
    GLuint SubtractGradient;
    GLuint ComputeDivergence;
    GLuint ApplyImpulse;
    GLuint ApplyBuoyancy;
} Programs;



typedef struct Surface_ {
    GLuint FboHandle;
    GLuint TextureHandle;
    int NumComponents;
} Surface;

typedef struct Slab_ {
    Surface Ping;
    Surface Pong;
} Slab;


typedef struct Vector2_ {
    int X;
    int Y;
} Vector2;



static GLuint QuadVao;
static GLuint VisualizeProgram;
static Slab Velocity, Density, Pressure, Temperature;
static Surface Divergence, Obstacles;



auto GetAnchorPoint = [](int background_width, int backgroud_height, int viewport_width, int viewport_height)
		{
			Vector2 point = { (background_width - viewport_width)/2, (backgroud_height - viewport_height)/2 };
			return point;
		};



GLuint CreateProgram(const GLchar * vs_src, const GLchar * gs_src, const GLchar * fs_src)
{
	//GL_CHECK_CONDITION(vs_src != 0, "Vertex shader is missing!");

    GLint	compiled;
    GLchar	compilerSpew[256];
    GLuint	programHandle = glCreateProgram();

    GLuint vsHandle = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vsHandle, 1, &vs_src, 0);
    glCompileShader(vsHandle);
    glGetShaderiv(vsHandle, GL_COMPILE_STATUS, &compiled);

    glGetShaderInfoLog(vsHandle, sizeof(compilerSpew), 0, compilerSpew);
    //GL_CHECK_CONDITION(compiled, compilerSpew);
    glAttachShader(programHandle, vsHandle);

    GLuint gsHandle;
    if (gs_src)
    {/*
        gsHandle = glCreateShader(GL_GEOMETRY_SHADER);

        glShaderSource(gsHandle, 1, &gs_src, 0);
        glCompileShader(gsHandle);
        glGetShaderiv(gsHandle, GL_COMPILE_STATUS, &compiled);
        glGetShaderInfoLog(gsHandle, sizeof(compilerSpew), 0, compilerSpew);
        GL_CHECK_CONDITION(compiled, compilerSpew);
        glAttachShader(programHandle, gsHandle);*/
    }

    GLuint fsHandle;
    if (fs_src)
    {
        fsHandle = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(fsHandle, 1, &fs_src, 0);
        glCompileShader(fsHandle);
        glGetShaderiv(fsHandle, GL_COMPILE_STATUS, &compiled);
        glGetShaderInfoLog(fsHandle, sizeof(compilerSpew), 0, compilerSpew);
        //GL_CHECK_CONDITION(compiled, compilerSpew);
        glAttachShader(programHandle, fsHandle);
    }

    glLinkProgram(programHandle);

    GLint linkSuccess;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linkSuccess);
    glGetProgramInfoLog(programHandle, sizeof(compilerSpew), 0, compilerSpew);

    if (!linkSuccess)
    {
        LOG_ERROR("GL link error", compilerSpew);
        if (vs_src) LOG_INFO("Vertex Shader:", vs_src);
        if (gs_src) LOG_INFO("Geometry Shader:", gs_src);
        if (fs_src) LOG_INFO("Fragment Shader:", fs_src);
    }

    return programHandle;
}





GLuint CreateQuad()
{
    short positions[] = {
        -1, -1,
         1, -1,
        -1,  1,
         1,  1,
    };

    // Create the VAO:
    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Create the VBO:
    GLuint vbo;
    GLsizeiptr size = sizeof(positions);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, positions, GL_STATIC_DRAW);

    // Set up the vertex layout:
    GLsizeiptr stride = 2 * sizeof(positions[0]);
    glEnableVertexAttribArray(PositionSlot);
    glVertexAttribPointer(PositionSlot, 2, GL_SHORT, GL_FALSE, stride, 0);

    return vao;
}






void CreateObstacles(Surface dest, int width, int height)
{
	PRINT_SEPARATOR();
    LOG_INFO("Width:", width);
    LOG_INFO("Height:", height);
    PRINT_SEPARATOR();

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
	glViewport(0, 0, width, height);
    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    GLuint program = CreateProgram(vs_fluid, 0, fs_fill);
    glUseProgram(program);

    const int DrawBorder = 1;
    if (DrawBorder)
    {
        //#define T 0.9999f
		#define W 0.9999f
		#define H 0.9999f
        float positions[] = { -W, -H, W, -H, W,  H, -W,  H, -W, -H };
        #undef W
		#undef H

        GLuint vbo;
        GLsizeiptr size = sizeof(positions);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, size, positions, GL_STATIC_DRAW);

        GLsizeiptr stride = 2 * sizeof(positions[0]);

        glEnableVertexAttribArray(PositionSlot);
        glVertexAttribPointer(PositionSlot, 2, GL_FLOAT, GL_FALSE, stride, 0);
        glDrawArrays(GL_LINE_STRIP, 0, 5);
        glDeleteBuffers(1, &vbo);
    }

    // Cleanup
    glDeleteProgram(program);
    glDeleteVertexArrays(1, &vao);
}





Surface CreateSurface(GLsizei width, GLsizei height, int numComponents)
{
    GLuint fboHandle;

    glGenFramebuffers(1, &fboHandle);
    glBindFramebuffer(GL_FRAMEBUFFER, fboHandle);

    GLuint textureHandle;

    glGenTextures(1, &textureHandle);
    glBindTexture(GL_TEXTURE_2D, textureHandle);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    const int UseHalfFloats = 1;
    if (UseHalfFloats)
    {
        switch (numComponents)
        {
            case 1: glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, width, height, 0, GL_RED, GL_HALF_FLOAT, 0); break;
            case 2: glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, width, height, 0, GL_RG, GL_HALF_FLOAT, 0); break;
            case 3: glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_HALF_FLOAT, 0); break;
            case 4: glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_HALF_FLOAT, 0); break;
            default: LOG_ERROR("Slab", "Illegal slab format.");
        }
    }
    else
    {
        switch (numComponents)
        {
            case 1: glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT, 0); break;
            case 2: glTexImage2D(GL_TEXTURE_2D, 0, GL_RG32F, width, height, 0, GL_RG, GL_FLOAT, 0); break;
            case 3: glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_FLOAT, 0); break;
            case 4: glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, 0); break;
            default: LOG_ERROR("Slab", "Illegal slab format.");
        }
    }

    //GL_CHECK_CONDITION(GL_NO_ERROR == glGetError(), "Unable to create normals texture");

    GLuint colorbuffer;

    glGenRenderbuffers(1, &colorbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, colorbuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureHandle, 0);

    //GL_CHECK_CONDITION(GL_NO_ERROR == glGetError(), "Unable to attach color buffer");
    //GL_CHECK_CONDITION(GL_FRAMEBUFFER_COMPLETE == glCheckFramebufferStatus(GL_FRAMEBUFFER), "Unable to create FBO.");

    Surface surface = { fboHandle, textureHandle, numComponents };

    glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    return surface;
}


Slab CreateSlab(GLsizei width, GLsizei height, int numComponents)
{
    Slab slab;

    slab.Ping = CreateSurface(width, height, numComponents);
    slab.Pong = CreateSurface(width, height, numComponents);

    return slab;
}


static void ResetState()
{
    glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_BLEND);
}



void InitSlabOps()
{
    Programs.Advect = CreateProgram(vs_fluid, 0, fs_advect);
    Programs.Jacobi = CreateProgram(vs_fluid, 0, fs_jacobi);
    Programs.SubtractGradient = CreateProgram(vs_fluid, 0, fs_subtract_gradient);
    Programs.ComputeDivergence = CreateProgram(vs_fluid, 0, fs_compute_divergence);
    Programs.ApplyImpulse = CreateProgram(vs_fluid, 0, fs_splat);
    Programs.ApplyBuoyancy = CreateProgram(vs_fluid, 0, fs_buoyancy);/**/
}

void SwapSurfaces(Slab* slab)
{
    Surface temp = slab->Ping;
    slab->Ping = slab->Pong;
    slab->Pong = temp;
}

void ClearSurface(Surface s, float v)
{
    glBindFramebuffer(GL_FRAMEBUFFER, s.FboHandle);
    glClearColor(v, v, v, v);
    glClear(GL_COLOR_BUFFER_BIT);
}

void Advect(Surface velocity, Surface source, Surface obstacles, Surface dest, float dissipation)
{
    GLuint p = Programs.Advect;
    glUseProgram(p);

    GLint inverseSize = glGetUniformLocation(p, "InverseSize");
    GLint timeStep = glGetUniformLocation(p, "TimeStep");
    GLint dissLoc = glGetUniformLocation(p, "Dissipation");
    GLint sourceTexture = glGetUniformLocation(p, "SourceTexture");
    GLint obstaclesTexture = glGetUniformLocation(p, "Obstacles");

    glUniform2f(inverseSize, 1.0f / TextureWidth(), 1.0f / TextureHeight());
    glUniform1f(timeStep, TimeStep);
    glUniform1f(dissLoc, dissipation);
    glUniform1i(sourceTexture, 1);
    glUniform1i(obstaclesTexture, 2);

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, velocity.TextureHandle);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, source.TextureHandle);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, obstacles.TextureHandle);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ResetState();
}

void Jacobi(Surface pressure, Surface divergence, Surface obstacles, Surface dest)
{
    GLuint p = Programs.Jacobi;
    glUseProgram(p);

    GLint alpha = glGetUniformLocation(p, "Alpha");
    GLint inverseBeta = glGetUniformLocation(p, "InverseBeta");
    GLint dSampler = glGetUniformLocation(p, "Divergence");
    GLint oSampler = glGetUniformLocation(p, "Obstacles");

    glUniform1f(alpha, -CellSize * CellSize);
    glUniform1f(inverseBeta, 0.25f);
    glUniform1i(dSampler, 1);
    glUniform1i(oSampler, 2);

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, pressure.TextureHandle);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, divergence.TextureHandle);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, obstacles.TextureHandle);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ResetState();
}

void SubtractGradient(Surface velocity, Surface pressure, Surface obstacles, Surface dest)
{
    GLuint p = Programs.SubtractGradient;
    glUseProgram(p);

    GLint gradientScale = glGetUniformLocation(p, "GradientScale");
    glUniform1f(gradientScale, GradientScale);
    GLint halfCell = glGetUniformLocation(p, "HalfInverseCellSize");
    glUniform1f(halfCell, 0.5f / CellSize);
    GLint sampler = glGetUniformLocation(p, "Pressure");
    glUniform1i(sampler, 1);
    sampler = glGetUniformLocation(p, "Obstacles");
    glUniform1i(sampler, 2);

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, velocity.TextureHandle);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, pressure.TextureHandle);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, obstacles.TextureHandle);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ResetState();
}

void ComputeDivergence(Surface velocity, Surface obstacles, Surface dest)
{
    GLuint p = Programs.ComputeDivergence;
    glUseProgram(p);

    GLint halfCell = glGetUniformLocation(p, "HalfInverseCellSize");
    glUniform1f(halfCell, 0.5f / CellSize);
    GLint sampler = glGetUniformLocation(p, "Obstacles");
    glUniform1i(sampler, 1);

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, velocity.TextureHandle);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, obstacles.TextureHandle);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ResetState();
}

void ApplyImpulse(Surface dest, Vector2 position, float value)
{
    GLuint p = Programs.ApplyImpulse;
    glUseProgram(p);

    GLint pointLoc = glGetUniformLocation(p, "Point");
    GLint radiusLoc = glGetUniformLocation(p, "Radius");
    GLint fillColorLoc = glGetUniformLocation(p, "FillColor");

//    PRINT_SEPARATOR();
//    LOG_INFO("Position X:", position.X);
//    LOG_INFO("Position Y:", position.Y);
//    LOG_INFO("SplatRadius:", SplatRadius());
//    PRINT_SEPARATOR();

    glUniform2f(pointLoc, (float) position.X, (float) position.Y);
    glUniform1f(radiusLoc, SplatRadius());
    glUniform3f(fillColorLoc, value, value, value);

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glEnable(GL_BLEND);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ResetState();
}

void ApplyBuoyancy(Surface velocity, Surface temperature, Surface density, Surface dest)
{
    GLuint p = Programs.ApplyBuoyancy;
    glUseProgram(p);

    GLint tempSampler = glGetUniformLocation(p, "Temperature");
    GLint inkSampler = glGetUniformLocation(p, "Density");
    GLint ambTemp = glGetUniformLocation(p, "AmbientTemperature");
    GLint timeStep = glGetUniformLocation(p, "TimeStep");
    GLint sigma = glGetUniformLocation(p, "Sigma");
    GLint kappa = glGetUniformLocation(p, "Kappa");

    glUniform1i(tempSampler, 1);
    glUniform1i(inkSampler, 2);
    glUniform1f(ambTemp, AmbientTemperature);
    glUniform1f(timeStep, TimeStep);
    glUniform1f(sigma, SmokeBuoyancy);
    glUniform1f(kappa, SmokeWeight);

    glBindFramebuffer(GL_FRAMEBUFFER, dest.FboHandle);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, velocity.TextureHandle);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, temperature.TextureHandle);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, density.TextureHandle);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    ResetState();
}





const char* FluidInitialize(int width, int height)
{
	ScreenWidth		= width;
	ScreenHeight	= height;

    int tw = TextureWidth();
    int th = TextureHeight();

    PRINT_SEPARATOR();
    LOG_INFO("ScreenWidth:", ScreenWidth);
	LOG_INFO("ScreenHeight:", ScreenHeight);
	LOG_INFO("ViewportWidth:", ViewportWidth);
	LOG_INFO("ViewportHeight:", ViewportHeight);
	PRINT_SEPARATOR();

    Velocity = CreateSlab(tw, th, 2);
    Density = CreateSlab(tw, th, 1);
    Pressure = CreateSlab(tw, th, 1);
    Temperature = CreateSlab(tw, th, 1);
    Divergence = CreateSurface(tw, th, 3);

    InitSlabOps();

    VisualizeProgram = CreateProgram(vs_fluid, 0, fs_visualize);

    Obstacles = CreateSurface(tw, th, 3);
    CreateObstacles(Obstacles, tw, th);

    QuadVao = CreateQuad();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    ClearSurface(Temperature.Ping, AmbientTemperature);

    return "Fluid";
}

auto F = [](float x, int amplitude, float period)
		{
			return amplitude * sin(2.0f * M_PI * x / period);
		};

void FluidUpdate(unsigned int elapsedMicroseconds)
{
	glBindVertexArray(QuadVao);

	int tw = TextureWidth();
	int th = TextureHeight();

    glViewport(0, 0, tw, th);

//    PRINT_SEPARATOR();
//    LOG_INFO("Width:", tw);
//    LOG_INFO("Height:", th);
//    PRINT_SEPARATOR();

	Advect(Velocity.Ping, Velocity.Ping, Obstacles, Velocity.Pong, VelocityDissipation);
    SwapSurfaces(&Velocity);

    Advect(Velocity.Ping, Temperature.Ping, Obstacles, Temperature.Pong, TemperatureDissipation);
    SwapSurfaces(&Temperature);

    Advect(Velocity.Ping, Density.Ping, Obstacles, Density.Pong, DensityDissipation);
    SwapSurfaces(&Density);

    ApplyBuoyancy(Velocity.Ping, Temperature.Ping, Density.Ping, Velocity.Pong);
    SwapSurfaces(&Velocity);

    static int X 	= 0;

    int amplitude	= tw / 3 / 2;
    int	period 		= amplitude * 8;
    int FX	= 0;
    int	y = 65;

	FX = F(X, amplitude, period);

//    	LOG_INFO("X:", X);
//    	LOG_INFO("FX:", FX);

	if ( ++X > period) X = 0;

    Vector2	ImpulsePositions[] = { { 0, y }, { tw / 2 + FX, y - 65}, { tw, y} };

    for ( int i = 0; i < sizeof(ImpulsePositions) / sizeof(ImpulsePositions[0]); i++)
    {
		ApplyImpulse(Temperature.Ping, ImpulsePositions[i], ImpulseTemperature);
		ApplyImpulse(Density.Ping, ImpulsePositions[i], ImpulseDensity);
    }

    ComputeDivergence(Velocity.Ping, Obstacles, Divergence);
    ClearSurface(Pressure.Ping, 0);

    for (int i = 0; i < NumJacobiIterations; ++i)
    {
        Jacobi(Pressure.Ping, Divergence, Obstacles, Pressure.Pong);
        SwapSurfaces(&Pressure);
    }

    SubtractGradient(Velocity.Ping, Pressure.Ping, Obstacles, Velocity.Pong);
    SwapSurfaces(&Velocity);

    //GL_CHECK_CONDITION(GL_NO_ERROR == glGetError(), "GL ERROR");
}


struct GradientColor
{
	float r, g, b;

	float delta;
	float r_delta;
	float g_delta;
	float b_delta;

	float lower_limit;
	float upper_limit;


	GradientColor() :
		delta(0.02f),
		lower_limit(0.1f),
		upper_limit(1.0f)
	{
		auto rand_color = []()
		{
			return (float)rand()/(float)RAND_MAX;
		};

		r = rand_color();
		g = rand_color();
		b = rand_color();

		InitDeltas();
	}

	void InitDeltas()
	{
		auto rand_delta = [&]()
		{
			return (rand() % 2 == 0 ? 1 : -1) * delta;
		};

	    r_delta = rand_delta();
	    g_delta = rand_delta();
	    b_delta = rand_delta();
	}

	void Update()
	{
		auto gradient = [&](float & v, float & delta)
		{
			v += delta;

			if (v > upper_limit)
			{
				v = upper_limit;
				InitDeltas();
				if (delta > 0.0) delta = -delta;
			}
			else if (v < lower_limit)
			{
				v = lower_limit;
				InitDeltas();
				if (delta < 0.0 ) delta = -delta;
			}
		};

		gradient(r, r_delta);
		gradient(g, g_delta);
		gradient(b, b_delta);
	}

	void Dump()
	{
		LOG_INFO("R:", r);
		LOG_INFO("G:", g);
		LOG_INFO("B:", b);
		LOG_INFO("R-Delta:", r_delta);
		LOG_INFO("G-Delta:", g_delta);
		LOG_INFO("B-Delta:", b_delta);
	}
} g_smoke_color, g_obstacle_color;


void FluidRender(GLuint windowFbo)
{
    // Bind visualization shader and set up blend state:
    glUseProgram(VisualizeProgram);

    GLint fillColor = glGetUniformLocation(VisualizeProgram, "FillColor");
    GLint scale = glGetUniformLocation(VisualizeProgram, "Scale");

    GLint enableOffset = glGetUniformLocation(VisualizeProgram, "EnableOffset");
    GLint offset = glGetUniformLocation(VisualizeProgram, "Offset");
    GLint viewport = glGetUniformLocation(VisualizeProgram, "Viewport");
    glEnable(GL_BLEND);
    glBindFramebuffer(GL_FRAMEBUFFER, windowFbo);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    int w = ViewportWidth;
    int h = ViewportHeight;

    Vector2 AnchorPoint = GetAnchorPoint(ScreenWidth, ScreenHeight, w, h);

    glViewport(AnchorPoint.X, AnchorPoint.Y, w, h);

    static int count = 0;

    if (++count > 6)
    {
    	count = 0;

		g_smoke_color.Update();
		g_obstacle_color.Update();

#if 0
		g_smoke_color.Dump();
		g_obstacle_color.Dump();
#endif
    }

	glBindVertexArray(QuadVao);

	float scale_x = 1.0f / (ViewportWidth * 1.0f);
	float scale_y = 1.0f / (ViewportHeight * 1.0f);

    glUniform2f(scale, scale_x, scale_y);
    glUniform1i(enableOffset, 1);
    glUniform2f(offset, AnchorPoint.X, AnchorPoint.Y);
    glUniform2f(viewport, ViewportWidth, ViewportHeight);

    // Draw smoke:
    glBindTexture(GL_TEXTURE_2D, Density.Ping.TextureHandle);
    glUniform3f(fillColor, g_smoke_color.r, g_smoke_color.g, g_smoke_color.b);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);


    // Disable blending:
    glDisable(GL_BLEND);
}

