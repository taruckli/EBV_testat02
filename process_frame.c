/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */
 
/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT
#define RGB_COLORS 0	//1 wenn mit RGB-Farben die ChangeDetektion ausgeführt werden soll

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

bool ManualThreshold;

/* skip pixel at border */
const int Border = 2;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

/* size of centroid marker */
const int SizeCross = 10;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

unsigned char OtsuThreshold(int InIndex);
void Binarize(unsigned char threshold);
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();


void ResetProcess()
{
	//called when "reset" button is pressed
	if(ManualThreshold == false)
		ManualThreshold = true;
	else
		ManualThreshold = false;
}


void ProcessFrame() {
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		ManualThreshold = false;
	} else {


		unsigned char Threshold = OtsuThreshold(SENSORIMG);

		Binarize(Threshold);

		Erode_3x3(THRESHOLD, INDEX0);
		Dilate_3x3(INDEX0, THRESHOLD);

		DetectRegions();

		DrawBoundingBoxes();
		ChangeDetection();

		if(ManualThreshold) {
			char Text[] = "manual threshold";
			DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);
		} else {
			char Text[] = " Otsu's threshold";
			DrawString(20, 20, strlen(Text), SMALL, CYAN, Text);
		}
	}
}


void ChangeDetection() {
	const int NumFgrCol = 2;
	uint8 FrgCol[2][3];

	if(RGB_COLORS){
		//Blau, Rot im BGR-Format
		FrgCol[0][0] = 154; //1. detektierte Farbe Blau-Wert
		FrgCol[0][1] = 125; //1. detektierte Farbe Grün-Wert
		FrgCol[0][2] = 81; //1. detektierte Farbe Rot-Wert
		FrgCol[1][0] = 75;	//2. detektierte Farbe Blau-Wert
		FrgCol[1][1] = 57;  //2. detektierte Farbe Grün-Wert
		FrgCol[1][2] = 234; //2. detektierte Farbe Rot-Wert

		int r, c, frg, p;
		memset(data.u8TempImage[INDEX0], 0, IMG_SIZE);
		memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);
		//loop over the rows
		for(r = 0; r < nr*nc; r += nc) {
			//loop over the columns
			for(c = 0; c < nc; c++) {
				//loop over the different Frg colors and find smallest difference
				int MinDif = 1 << 30;
				int MinInd = 0;
				for(frg = 0; frg < NumFgrCol; frg++) {
					int Dif = 0;
					//loop over the color planes (r, g, b) and sum up the difference
					for(p = 1; p < NUM_COLORS; p++) {
						Dif += abs((int) data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+p]-
						(int) FrgCol[frg][p]);
					}
					if(Dif < MinDif) {
						MinDif = Dif;
						MinInd = frg;
					}
				}
				//if the difference is smaller than threshold value
				if(MinDif < data.ipc.state.nThreshold) {
					//set pixel value to 255 in THRESHOLD image for further processing
					//(we use only the first third of the image buffer)
					data.u8TempImage[INDEX1][(r+c)] = 255;
					//set pixel value to Frg color in BACKGROUND image for visualization
					for(p = 0; p < NUM_COLORS; p++) {
						data.u8TempImage[BACKGROUND][(r+c)*NUM_COLORS+p] = FrgCol[MinInd][p];
					}
				}
			}
		}
	} else {
		FrgCol[0][0] = 100;
		FrgCol[0][1] = 100;
		FrgCol[0][2] = 214;
		FrgCol[1][0] = 100;
		FrgCol[1][1] = 160;
		FrgCol[1][2] = 89;

		ycbcr();

		int r, c, frg, p;
		memset(data.u8TempImage[INDEX0], 0, IMG_SIZE);
		memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);
		//loop over the rows
		for(r = 0; r < nr*nc; r += nc) {
			//loop over the columns
			for(c = 0; c < nc; c++) {
				//loop over the different Frg colors and find smallest difference
				int MinDif = 1 << 30;
				int MinInd = 0;
				for(frg = 0; frg < NumFgrCol; frg++) {
					int Dif = 0;
					//loop over the color planes (r, g, b) and sum up the difference
					for(p = 1; p < NUM_COLORS; p++) {
						Dif += abs((int) data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+p]-
						(int) FrgCol[frg][p]);
					}
					if(Dif < MinDif) {
						MinDif = Dif;
						MinInd = frg;
					}
				}
				//if the difference is smaller than threshold value
				if(MinDif < data.ipc.state.nThreshold) {
					//set pixel value to 255 in THRESHOLD image for further processing
					//(we use only the first third of the image buffer)
					data.u8TempImage[INDEX1][(r+c)] = 255;
					//set pixel value to Frg color in BACKGROUND image for visualization
					for(p = 0; p < NUM_COLORS; p++) {
						data.u8TempImage[BACKGROUND][(r+c)*NUM_COLORS+p] = FrgCol[MinInd][p];
					}
				}
			}
		}
	}
}

void ycbcr(){
	for(int r = 0; r < nr*nc; r += nc) {
		//loop over the columns
		for(int c = 0; c < nc; c++) {
			//get rgb values (order is actually bgr!)
			float B_ = data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+0];
			float G_ = data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+1];
			float R_ = data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+2];
			uint8 Y_ = (uint8) ( 0 + 0.299*R_ + 0.587*G_ + 0.114*B_);
			uint8 Cb_ = (uint8) (128 - 0.169*R_ - 0.331*G_ + 0.500*B_);
			uint8 Cr_ = (uint8) (128 + 0.500*R_ - 0.419*G_ - 0.081*B_);
			//we write result to THRESHOLD
			data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+0] = Y_;
			data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+1] = Cb_;
			data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+2] = Cr_;
		}
	}
}


void Binarize(unsigned char threshold) {
	int r, c;
	//set result buffer to zero
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);

	//loop over the rows
	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		//loop over the columns
		for(c = Border; c < (nc-Border); c++) {
			//manual threshold?
			if(ManualThreshold) {
				if(data.u8TempImage[SENSORIMG][r+c] < data.ipc.state.nThreshold) {
					data.u8TempImage[THRESHOLD][r+c] = 255;
				}
			} else {
				if(data.u8TempImage[SENSORIMG][r+c] < threshold) {
					data.u8TempImage[THRESHOLD][r+c] = 255;
				}
			}
		}
	}
}


unsigned char OtsuThreshold(int InIndex) {
	//first part: extract gray value histogram
	unsigned int i1, best_i, K;
	float Hist[256];
	unsigned char* p = data.u8TempImage[InIndex];
	float best;
	memset(Hist, 0, sizeof(Hist));

	for(i1 = 0; i1 < nr*nc; i1++) {
		Hist[p[i1]] += 1;
	}
	//second part: determine threshold according to Otsu's method
	best = 0;
	best_i = 0;
	for(K = 0; K < 255; K++)	{
		//the class accumulators
		float w0 = 0, mu0 = 0, w1 = 0, mu1 = 0;
		float bestloc;

		//class 0 and 1 probabilities and averages
		for(i1 = 0; i1 <= K; i1++)	{
			w0 += Hist[i1];
			mu0 += (Hist[i1]*i1);
		}
		for(; i1 <= 255; i1++) {
			w1 += Hist[i1];
			mu1 += (Hist[i1]*i1);
		}
		//do normalization of average values
		mu0 /= w0;
		mu1 /= w1;

		bestloc = w0*w1*(mu0-mu1)*(mu0-mu1);
		if(bestloc > best) {
			best = bestloc;
			best_i = K;
			//OscLog(INFO, "%d %d %d %d %d\n", i1, w0, w1, mu0, mu1);
		}
	}
	//OscLog(INFO, "%d %f\n", best_i, best);
	return (unsigned char) best_i;
}

void Erode_3x3(int InIndex, int OutIndex)
{
	int c, r;

	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		for(c = Border; c < (nc-Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r+c];
			data.u8TempImage[OutIndex][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
											   *(p-1)    & *p      & *(p+1)    &
											   *(p+nc-1) & *(p+nc) & *(p+nc+1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex)
{
	int c, r;

	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		for(c = Border; c < (nc-Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r+c];
			data.u8TempImage[OutIndex][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
											        *(p-1)    | *p      | *(p+1)    |
											        *(p+nc-1) | *(p+nc) | *(p+nc+1);
		}
	}
}


void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for(i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary( &Pic, &ImgRegions);
	OscVisGetRegionProperties( &ImgRegions);
}


void DrawBoundingBoxes() {
	uint16 o;
	for(o = 0; o < ImgRegions.noOfObjects; o++) {
		if(ImgRegions.objects[o].area > MinArea) {
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft, ImgRegions.objects[o].bboxTop,
							ImgRegions.objects[o].bboxRight, ImgRegions.objects[o].bboxBottom, false, GREEN);

			DrawLine(ImgRegions.objects[o].centroidX-SizeCross, ImgRegions.objects[o].centroidY,
					 ImgRegions.objects[o].centroidX+SizeCross, ImgRegions.objects[o].centroidY, RED);
			DrawLine(ImgRegions.objects[o].centroidX, ImgRegions.objects[o].centroidY-SizeCross,
								 ImgRegions.objects[o].centroidX, ImgRegions.objects[o].centroidY+SizeCross, RED);

		}
	}
}


