// This source code is property of the Computer Graphics and Visualization chair of the
// TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
// The main file of the plugin. It defines a class that demonstrates how to register with
// the scene graph, drawing primitives, creating a GUI, using a config file and various
// other parts of the framework.

// Framework core
#include <cgv/base/register.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/trigger.h>
#include <cgv/render/drawable.h>
#include <cgv/render/render_types.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/texture.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/vertex_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv/media/font/font.h>
#include <cgv/math/ftransform.h>

// Framework standard plugins
#include <cgv_gl/gl/gl.h>

// Some constant symbols
#define FB_MAX_RESOLUTION 2048

// The CGV framework demonstration class
class live_earth
	: public cgv::base::base,      // This class supports reflection
	  public cgv::gui::provider,   // Instances of this class provde a GUI
	  public cgv::render::drawable // Instances of this class can be rendered
{
protected:
	cgv::render::texture EarthMap;
	cgv::render::shader_program gfs, gfa, sfs, sfa;
	GLuint VerticesVBO, TexCoordsVBO, VerticesCount;
	float InnerRadius, OuterRadius;
	
public:
	live_earth() {
		
	}

	bool init(cgv::render::context & ctx) {
        bool Error = false;

        Error |= !EarthMap.create_from_image(ctx,
            "images/earth.tga");
        //Error |= !LightsMap.LoadTexture2D("lightsmap.jpg");
        Error |= !gfs.attach_dir(ctx, "shaders/groundfromspace/",true);
        Error |= !gfa.attach_dir(ctx, "shaders/groundfromatmosphere/", true);
        Error |= !sfs.attach_dir(ctx, "shaders/skyfromspace/", true);
        Error |= !sfa.attach_dir(ctx, "shaders/skyfromatmosphere/", true);

        if (Error)
        {
            return;
        }

        float Kr = 0.0030f;
        float Km = 0.0015f;
        float ESun = 16.0f;
        float g = -0.75f;
        InnerRadius = 10.0f * 25.0f;
        OuterRadius = 10.25f * 25.0f;
        float Scale = 1.0f / (OuterRadius - InnerRadius);
        float ScaleDepth = 0.25f;
        float ScaleOverScaleDepth = Scale / ScaleDepth;

        cgv::render::shader_program programs[] = { gfs, gfa, sfs, sfa };

        for (int i = 0; i < 4; i++)
        {
            glUseProgram(programs[i]);
            glUniform3f(glGetUniformLocation(programs[i], "v3LightPos"), 0.0f, 0.0f, 1.0f);
            glUniform3f(glGetUniformLocation(programs[i], "v3InvWavelength"), 1.0f / powf(0.650f, 4.0f), 1.0f / powf(0.570f, 4.0f), 1.0f / powf(0.475f, 4.0f));
            glUniform1f(glGetUniformLocation(programs[i], "fInnerRadius"), InnerRadius);
            glUniform1f(glGetUniformLocation(programs[i], "fInnerRadius2"), InnerRadius * InnerRadius);
            glUniform1f(glGetUniformLocation(programs[i], "fOuterRadius"), OuterRadius);
            glUniform1f(glGetUniformLocation(programs[i], "fOuterRadius2"), OuterRadius * OuterRadius);
            glUniform1f(glGetUniformLocation(programs[i], "fKrESun"), Kr * ESun);
            glUniform1f(glGetUniformLocation(programs[i], "fKmESun"), Km * ESun);
            glUniform1f(glGetUniformLocation(programs[i], "fKr4PI"), Kr * 4.0f * (float)M_PI);
            glUniform1f(glGetUniformLocation(programs[i], "fKm4PI"), Km * 4.0f * (float)M_PI);
            glUniform1f(glGetUniformLocation(programs[i], "fScale"), Scale);
            glUniform1f(glGetUniformLocation(programs[i], "fScaleDepth"), ScaleDepth);
            glUniform1f(glGetUniformLocation(programs[i], "fScaleOverScaleDepth"), ScaleOverScaleDepth);
            glUniform1f(glGetUniformLocation(programs[i], "g"), g);
            glUniform1f(glGetUniformLocation(programs[i], "g2"), g * g);
            glUniform1i(glGetUniformLocation(programs[i], "Samples"), 4);
            glUniform1i(glGetUniformLocation(programs[i], "s2Tex1"), 0);
            glUniform1i(glGetUniformLocation(programs[i], "s2Tex2"), 1);
            //glUniform1i(glGetUniformLocation(programs[i], "s2Tex3"), 2);
            glUseProgram(0);
        }

        int X = 128, Y = X / 2, vpos = 0, tpos = 0;
        float a, stepa = (float)M_PI * 2.0f / (float)X, stepb = (float)M_PI / (float)Y, b = -(float)M_PI_2 + stepb;

        vec3* vertices = new vec3[X * (Y - 1)];

        for (int y = 0; y < (Y - 1); y++)
        {
            a = -(float)M_PI;

            for (int x = 0; x < X; x++)
            {
                vertices[y * X + x] = normalize(vec3(sin(a) * cos(b), sin(b), cos(a) * cos(b)));
                a += stepa;
            }

            b += stepb;
        }

        VerticesCount = (X * (Y - 2) * 2 + X * 2) * 3;

        vec3* Vertices = new vec3[VerticesCount];
        vec2* TexCoords = new vec2[VerticesCount];

        for (int x = 0; x < X; x++)
        {
            Vertices[vpos++] = vec3(0.0f, -1.0f, 0.0f);
            Vertices[vpos++] = vertices[(0 + 0) * X + ((x + 1) % X)];
            Vertices[vpos++] = vertices[(0 + 0) * X + ((x + 0) % X)];

            TexCoords[tpos++] = vec2((float)(x + 0.5f) / (float)X, 0.0f);
            TexCoords[tpos++] = vec2((float)(x + 1) / (float)X, (float)(0 + 1) / (float)Y);
            TexCoords[tpos++] = vec2((float)(x + 0) / (float)X, (float)(0 + 1) / (float)Y);
        }

        for (int y = 0; y < Y - 2; y++)
        {
            for (int x = 0; x < X; x++)
            {
                Vertices[vpos++] = vertices[(y + 0) * X + ((x + 0) % X)];
                Vertices[vpos++] = vertices[(y + 0) * X + ((x + 1) % X)];
                Vertices[vpos++] = vertices[(y + 1) * X + ((x + 1) % X)];

                TexCoords[tpos++] = vec2((float)(x + 0) / (float)X, (float)(1 + y + 0) / (float)Y);
                TexCoords[tpos++] = vec2((float)(x + 1) / (float)X, (float)(1 + y + 0) / (float)Y);
                TexCoords[tpos++] = vec2((float)(x + 1) / (float)X, (float)(1 + y + 1) / (float)Y);

                Vertices[vpos++] = vertices[(y + 1) * X + ((x + 1) % X)];
                Vertices[vpos++] = vertices[(y + 1) * X + ((x + 0) % X)];
                Vertices[vpos++] = vertices[(y + 0) * X + ((x + 0) % X)];

                TexCoords[tpos++] = vec2((float)(x + 1) / (float)X, float(1 + y + 1) / (float)Y);
                TexCoords[tpos++] = vec2((float)(x + 0) / (float)X, float(1 + y + 1) / (float)Y);
                TexCoords[tpos++] = vec2((float)(x + 0) / (float)X, float(1 + y + 0) / (float)Y);
            }
        }

        for (int x = 0; x < X; x++)
        {
            Vertices[vpos++] = vertices[(Y - 2) * X + ((x + 0) % X)];
            Vertices[vpos++] = vertices[(Y - 2) * X + ((x + 1) % X)];
            Vertices[vpos++] = vec3(0.0f, 1.0f, 0.0f);

            TexCoords[tpos++] = vec2((float)(x + 0) / (float)X, (float)(Y - 1) / (float)Y);
            TexCoords[tpos++] = vec2((float)(x + 1) / (float)X, (float)(Y - 1) / (float)Y);
            TexCoords[tpos++] = vec2((float)(x + 0.5f) / (float)X, 1.0f);
        }

        glGenBuffers(1, &VerticesVBO);
        glBindBuffer(GL_ARRAY_BUFFER, VerticesVBO);
        glBufferData(GL_ARRAY_BUFFER, VerticesCount * 3 * 4, Vertices, GL_STATIC_DRAW);

        glGenBuffers(1, &TexCoordsVBO);
        glBindBuffer(GL_ARRAY_BUFFER, TexCoordsVBO);
        glBufferData(GL_ARRAY_BUFFER, VerticesCount * 2 * 4, TexCoords, GL_STATIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, 0);

        delete[] vertices;
        delete[] Vertices;
        delete[] TexCoords;

        Camera.Look(vec3(0.0f, 0.0f, 768.0f), vec3(0.0f, 0.0f, 0.0f), InnerRadius);

        return true;
	}
	
};

// Create an instance of the demo class at plugin load and register it with the framework
cgv::base::object_registration<live_earth> cgv_demo_registration("");

// The following could be used to register the class with the framework but NOT create it
// upon plugin load. Instead, the user can create an instance from the application menu.
// However, config files are not straight-forward to use in this case, which is why we
// go for the method above.
/*
	cgv::base::factory_registration<cgv_demo> cgv_demo_factory(
		"new/cgv_demo", // menu path
		'D',            // the shortcut - capital D means ctrl+d
		true            // whether the class is supposed to be a singleton
	);
*/
