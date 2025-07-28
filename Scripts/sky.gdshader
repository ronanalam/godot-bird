shader_type sky;

uniform float testVar = 0.5;

vec3 EYEDIR_remap(vec3 eyedir) {
	return 0.5*(eyedir+vec3(1.0,1.0,1.0));
}

float EYEDIR_remap(float eyedirx) {
	return 0.5*(eyedirx+1.0);
}

float circle(vec2 r, vec2 center, float radius) {
	return 1.0 - step( radius, length(r-center) );
}

float circle(vec3 r, vec3 center, float radius) {
	return 1.0 - step( radius, acos(dot(r,center)) );
}

void sky() {
	// Called for every visible pixel in the sky background, as well as all pixels
	// in the radiance cubemap.
	//COLOR = vec3(16.0*(0.25-EYEDIR.y)*(0.25+EYEDIR.y), (1.0/0.15)*(0.15-EYEDIR.y)*(0.15+EYEDIR.y), EYEDIR.y);
	//COLOR = sin(0.01*PI*EYEDIR_remap(EYEDIR));
	COLOR = vec3( 0.25*vec2(circle(EYEDIR_remap(EYEDIR), vec3(1.0, 0.0, 0.0), 0.2)), 0.15);
}
