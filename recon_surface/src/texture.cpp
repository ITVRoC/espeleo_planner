#include "recon_surface/texture.hpp"



void texture::idw_interpolation(double xyz[], unsigned char rgb[]) 
{
	int nb_points = 6;
	double e = 1.5;
	double drgb[3];
	Distance tr_dist;
	double dist;

	Point_3 p(xyz[0],xyz[1],xyz[2]);
	K_neighbor_search search(tree, p, nb_points);

    std::vector<std::pair<size_t,double> > neighbors; //array idx, distance

    for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++)
    {
    	dist = tr_dist.inverse_of_transformed_distance(it->second);

    	if(dist == 0)
    		dist = 1e-8;

    	neighbors.push_back(std::make_pair(boost::get<1>(it->first),dist));
    }

 	drgb[0] =0; drgb[1] =0; drgb[2] =0;
    double weights = 0;

    for(int i=0; i<nb_points; i++)
    {
    	for(int j=0;j<3;j++)
    		drgb[j]+=colors[neighbors[i].first][j]*(1.0/pow(neighbors[i].second,e));

    	weights+=(1.0/pow(neighbors[i].second,e));
    }

    for(int i=0;i<3;i++)
    {
   		drgb[i]/=weights;
   		rgb[i] = (unsigned char)drgb[i];
    }

}

void texture::draw_triangle_up(int x, int y, int t, unsigned char rgb[], int sep)
{
	double xyz[3];
	Eigen::VectorXd a(3); a(0) = vertices[faces[uv_idx].v1].x; a(1) = vertices[faces[uv_idx].v1].y; a(2) = vertices[faces[uv_idx].v1].z;
	Eigen::VectorXd b(3); b(0) = vertices[faces[uv_idx].v2].x; b(1) = vertices[faces[uv_idx].v2].y; b(2) = vertices[faces[uv_idx].v2].z;
	Eigen::VectorXd c(3); c(0) = vertices[faces[uv_idx].v3].x; c(1) = vertices[faces[uv_idx].v3].y; c(2) = vertices[faces[uv_idx].v3].z;

	Eigen::VectorXd ac = c-a;
	Eigen::VectorXd bc = c-b;

	for(int i=0;i<t;i++)
	{
		//interpolate 3d positions inside face
		Eigen::VectorXd c_ac = a + (ac*((i-sep)/(double)(t-1/*-sep*/-3*sep)));
		Eigen::VectorXd c_bc = b + (bc*((i-sep)/(double)(t-1/*-sep*/-3*sep)));

		Eigen::VectorXd acbc = c_bc - c_ac;

		for(int j=i; j<t; j++)
		{
			Eigen::VectorXd c_acbc;

			if(t-i-1-2*sep!=0)
				c_acbc = c_ac + (acbc*((j-i-sep)/(double)(t-i-1-2*sep)));
			else
				c_acbc = c_ac; //+ (acbc*((j-i-sep)/0.5));

			xyz[0] = c_acbc(0); xyz[1] = c_acbc(1); xyz[2] = c_acbc(2);
			idw_interpolation(xyz,rgb);		

			img.at(y+i,x+j,0) = rgb[0];
			img.at(y+i,x+j,1) = rgb[1];
			img.at(y+i,x+j,2) = rgb[2];

			//cout<<"R:"<<(int)rgb[0]<<" G:"<<(int)rgb[1]<<" B:"<<(int)rgb[2]<<endl; getchar();
		}
	}

	//parametrize the face vertices	
	//upper triangle
	faces[uv_idx].uv1.x = (x+sep*2)/(double)texture_size; 
	faces[uv_idx].uv1.y = (y+sep)/(double)texture_size; 

	faces[uv_idx].uv2.x = (t+x-sep)/(double)texture_size; //rect angle
	faces[uv_idx].uv2.y = (y+sep)/(double)texture_size;

	faces[uv_idx].uv3.x = (t+x+-sep)/(double)texture_size; 
	faces[uv_idx++].uv3.y = (y+t-sep*2)/(double)texture_size;

	

}

void texture::draw_triangle_down(int x, int y, int t, unsigned char rgb[], int sep)
{

	double xyz[3];
	Eigen::VectorXd a(3); a(0) = vertices[faces[uv_idx].v1].x; a(1) = vertices[faces[uv_idx].v1].y; a(2) = vertices[faces[uv_idx].v1].z;
	Eigen::VectorXd b(3); b(0) = vertices[faces[uv_idx].v3].x; b(1) = vertices[faces[uv_idx].v3].y; b(2) = vertices[faces[uv_idx].v3].z;
	Eigen::VectorXd c(3); c(0) = vertices[faces[uv_idx].v2].x; c(1) = vertices[faces[uv_idx].v2].y; c(2) = vertices[faces[uv_idx].v2].z;

	Eigen::VectorXd ac = c-a;
	Eigen::VectorXd ab = b-a;

	for(int i=0;i<t;i++)
	{
		//interpolate 3d positions inside face
		Eigen::VectorXd c_ac = a + (ac*((i-2*sep)/(double)(t-1-3*sep/*-sep*/)));
		Eigen::VectorXd c_ab = a + (ab*((i-2*sep)/(double)(t-1-3*sep/*-sep*/)));

		Eigen::VectorXd acab = c_ab - c_ac;

		for(int j=0; j<=i; j++)
		{
			Eigen::VectorXd c_acab;

			if(i-2*sep!=0)
				c_acab = c_ac + (acab*((j-sep)/(double)(i-2*sep)));
			else
				c_acab = c_ac; // + (acab*((j-sep)/-0.5));

			xyz[0] = c_acab(0); xyz[1] = c_acab(1); xyz[2] = c_acab(2);
			idw_interpolation(xyz,rgb);	

			img.at(y+i,x+j,0) = rgb[0];
			img.at(y+i,x+j,1) = rgb[1];
			img.at(y+i,x+j,2) = rgb[2];

		}

	}

	//parametrize the face vertices	
	//lower triangle
	faces[uv_idx].uv1.x = (x+sep)/(double)texture_size; 
	faces[uv_idx].uv1.y = (y+sep*2)/(double)texture_size; 

	faces[uv_idx].uv2.x = (x+sep)/(double)texture_size; //rect angle
	faces[uv_idx].uv2.y = (y+t-sep)/(double)texture_size;

	faces[uv_idx].uv3.x = (t+x-sep*2)/(double)texture_size; 
	faces[uv_idx++].uv3.y = (y+t-sep)/(double)texture_size;
	
}

void texture::draw_quad_and_parametrize(int x, int y, int t, int n)
{
	unsigned char rgb[3];
	int size = t/n;
	int sep = size/5; //separation space for triangles in pixels
	sep = min(sep,6);

	//while(size < 2*sep && sep > 0)
	//	sep--;

	for(int i=0;i<n;i++)
			for(int j=0;j<n;j++)
			{
				//rgb[0] = rand()%256; rgb[1] = rand()%256; rgb[2] = rand()%256;
				//rgb[0] = 255; rgb[1] = 255; rgb[2] = 255;
				if(uv_idx < faces.size())
					draw_triangle_up(j*size+x,i*size+y,size,rgb,sep);

				//rgb[0] = rand()%256; rgb[1] = rand()%256; rgb[2] = rand()%256;
				if(uv_idx < faces.size())
					draw_triangle_down(j*size+x,i*size+y,size,rgb,sep);
			}
}


void texture::build_png()
{	
	size_t nfaces = faces.size();
	size_t num_large = nfaces/3 + nfaces%3;
	size_t num_medium = nfaces/3;
	size_t num_small = nfaces/3; 
	size_t acc=0;

	size_t nb_squares = num_large/2 + num_medium/8 + num_small/18;
	size_t l = sqrt(nb_squares) + 1;
	double t = 1.0/(double)l;

	int t_px = t*(texture_size);

	cout<<"Number of faces of the mesh: "<<nfaces<<endl;
	//cout<<"nb of squares: "<<nb_squares<<endl;
	cout<<"Created a grid of "<<l<<" by "<<l<<" for per-face uv parametrization."<<endl;
	//cout<<"t = "<<t<<endl;
	cout<<"Grid square size (pixels): "<<t*texture_size<<endl;

	int cont=0;

	for(int i=0; i< l;i++)
	{
		for(int j=0; j< l;j++)
		{	
			if(acc <num_large)
			{
				draw_quad_and_parametrize(j*t_px,i*t_px,t_px,1);
				acc+=2;
			}
			else if(acc < num_large + num_medium)
			{
				draw_quad_and_parametrize(j*t_px,i*t_px,t_px,2);
				acc+=8;
			}
			else
			{
				draw_quad_and_parametrize(j*t_px,i*t_px,t_px,3);
				acc+=18;
			}

		}
		printf("\rBaking texture... (%.2f%% done).",(i/(float)(l-1))*100.0); fflush(stdout);
	}
	printf("\n");


	cout<<"Writing texture: "<< output_path+"_texture.png" << endl;
	img.write(output_path+"_texture.png");

}

void texture::save_obj()
{
	ofstream f(output_path);

	string mtl_path = output_path+".mtl";
	string texture_path = output_path+"_texture.png";

	ofstream fmtl(mtl_path);

	string mtl_name = mtl_path.substr(mtl_path.rfind("/")+1);
	string texture_name = texture_path.substr(texture_path.rfind("/")+1);

	fmtl<<"newmtl material_0"<<endl<<
	"Ka 0.200000 0.200000 0.200000"<<endl<<
	"Kd 1.000000 1.000000 1.000000"<<endl<<
	"Ks 1.000000 1.000000 1.000000"<<endl<<
	"Tr 1.000000"<<endl<<
	"illum 2"<<endl<<
	"Ns 0.000000"<<endl<<
	"map_Kd " << texture_name << endl;

	f<<"# Generated by surface_recon software (ITV SpeleoRobo project - mesh reconstruction software) "<<endl<<endl;
	f<<"mtllib ./"<< mtl_name <<endl<<endl;

	cout<<"Saving obj file..."<<endl;

	for(size_t i=0; i< vertices.size(); i++)
		f<<"v "<<vertices[i].x<<" "<<vertices[i].y<<" "<<vertices[i].z<<endl;

	for(size_t i=0; i< faces.size(); i++)
	{
		f<<"vt "<<faces[i].uv1.x <<" "<<1.0-faces[i].uv1.y<<endl;
		f<<"vt "<<faces[i].uv2.x <<" "<<1.0-faces[i].uv2.y<<endl;
		f<<"vt "<<faces[i].uv3.x <<" "<<1.0-faces[i].uv3.y<<endl;
	}

	for(size_t i=0; i< faces.size(); i++)
		f<<"f "<<faces[i].v1+1<<"/"<<3*i+1<<" "<<faces[i].v2+1<<"/"<<3*i+2<<" "<<faces[i].v3+1<<"/"<<3*i+3<<endl;

	f.close();

	cout<<"Done saving obj file."<<endl;
	
}

double texture::face_area(const face& f)
{
	double x1,x2,x3,y1,y2,y3;
	size_t v1,v2,v3;

	v1 = f.v1; v2 = f.v2; v3 = f.v3;

	x1 = vertices[v1].x - vertices[v2].x; y1 = vertices[v1].x - vertices[v3].x;
	x2 = vertices[v1].y - vertices[v2].y; y2 = vertices[v1].y - vertices[v3].y;
	x3 = vertices[v1].z - vertices[v2].z; y3 = vertices[v1].z - vertices[v3].z;

	return (sqrt(pow(x2*y3-x3*y2,2)+pow(x3*y1-x1*y3,2)+pow(x1*y2-x2*y1,2)))/2.0;
}

bool texture::sort_face_by_area(const face& f1, const face& f2)
{
	return face_area(f1) >  face_area(f2);
}

void texture::read_off(string filename)
{
	ifstream f(filename.c_str());

	string header;
	size_t n_vertices, n_faces, n_edges;
	int n_poly;

	f>>header>>n_vertices>>n_faces>>n_edges;
	//cout<<header<<" "<<n_vertices<<" "<<n_faces<<" "<<n_edges<<endl;

	vertices.reserve(n_vertices);
	faces.reserve(n_faces);

	double x,y,z;
	size_t v1,v2,v3;

	for(size_t i=0; i<n_vertices;i++)
	{
		f>>x>>y>>z;
		vertices.push_back(vertex(x,y,z));
	}

	for(size_t i=0; i<n_faces;i++)
	{
		f>>n_poly>>v1>>v2>>v3;
		faces.push_back(face(v1,v2,v3));
		faces[i].area = face_area(faces[i]);
	}

	std::sort(faces.begin(),faces.end(),[](face f1, face f2){return f1.area > f2.area;}); //sort faces by decreasing area

	#ifdef DEBUG_TRUE
		for(int i=0;i<10;i++)
		{
			cout<<vertices[i].x <<" "<< vertices[i].y <<" "<< vertices[i].z<<endl;
			cout<<faces[i].v1 <<" "<< faces[i].v2 <<" " << faces[i].v3 << endl; 
			cout<<"face area:" << face_area(faces[i]) << endl;
		}
	

	cout<<"n_faces: "<<n_faces <<" faces: "<<faces.size()<<endl;
	cout<<"n_vertices: "<<n_vertices<<" vertices: "<<vertices.size()<<endl;
	#endif
}