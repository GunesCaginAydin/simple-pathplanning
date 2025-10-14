function [redObjectsMask, greenObjectsMask, blueObjectsMask] = seperate_colors(rgbImage)

    redChannel = rgbImage(:,:,1);
    greenChannel = rgbImage(:,:,2);
    blueChannel = rgbImage(:,:,3);

    allBlack = zeros(size(rgbImage, 1), size(rgbImage, 2), 'uint8');

    % red = cat(3, redChannel, allBlack, allBlack);
    % green = cat(3, allBlack, greenChannel, allBlack);
    % blue = cat(3, allBlack, allBlack, blueChannel);

    redThresholdLow1 = graythresh(redChannel);		
    redThresholdHigh1 = 255;
	greenThresholdLow1 = 0;
	greenThresholdHigh1 = graythresh(greenChannel);
	blueThresholdLow1 = 0;
	blueThresholdHigh1 = graythresh(blueChannel);
	redThresholdLow1 = uint8(redThresholdLow1 * 255);
	greenThresholdHigh1 = uint8(greenThresholdHigh1* 255);
	blueThresholdHigh1 = uint8(blueThresholdHigh1 * 255); 

    greenThresholdLow2 = graythresh(redChannel);		
    greenThresholdHigh2 = 255;
	redThresholdLow2 = 0;
	redThresholdHigh2 = graythresh(greenChannel);
	blueThresholdLow2 = 0;
	blueThresholdHigh2 = graythresh(blueChannel);
	greenThresholdLow2 = uint8(greenThresholdLow2 * 255);
	redThresholdHigh2 = uint8(redThresholdHigh2 * 255);
	blueThresholdHigh2 = uint8(blueThresholdHigh2 * 255); 

    redThresholdLow3 = graythresh(redChannel);		
    redThresholdHigh3 = 255;
	greenThresholdLow3 = 0;
	greenThresholdHigh3 = graythresh(greenChannel);
	blueThresholdLow3 = 0;
	blueThresholdHigh3 = graythresh(blueChannel);
	redThresholdLow3 = uint8(redThresholdLow3 * 255);
	greenThresholdHigh3 = uint8(greenThresholdHigh3 * 255);
	blueThresholdHigh3 = uint8(blueThresholdHigh3 * 255); 
    
    % redThresholdLow1 = 0; redThresholdLow2 = 0; redThresholdLow3 = 0;
    % redThresholdHigh1 = 255; redThresholdHigh2 = 70; redThresholdHigh3 = 70;
	% greenThresholdLow1 = 0; greenThresholdLow2 = 85; greenThresholdLow3 = 0;
	% greenThresholdHigh1 = 70; greenThresholdHigh2 = 0; greenThresholdHigh3 = 80;
	% blueThresholdLow1 = 0; blueThresholdLow2 = 0; blueThresholdLow3 = 85;
	% blueThresholdHigh1 = 90; blueThresholdHigh2 = 80; blueThresholdHigh3 = 255;

    redMask1 = (redChannel >= redThresholdLow1) & (redChannel <= redThresholdHigh1);
	greenMask1 = (greenChannel >= greenThresholdLow1) & (greenChannel <= greenThresholdHigh1);
	blueMask1 = (blueChannel >= blueThresholdLow1) & (blueChannel <= blueThresholdHigh1);

    redMask2 = (redChannel >= greenThresholdLow2) & (redChannel <= greenThresholdHigh2);
	greenMask2 = (greenChannel >= redThresholdLow2) & (greenChannel <= redThresholdHigh2);
	blueMask2 = (blueChannel >= blueThresholdLow2) & (blueChannel <= blueThresholdHigh2);

    redMask3 = (redChannel >= redThresholdLow3) & (redChannel <= redThresholdHigh3);
	greenMask3 = (greenChannel >= greenThresholdLow3) & (greenChannel <= greenThresholdHigh3);
	blueMask3 = (blueChannel >= blueThresholdLow3) & (blueChannel <= blueThresholdHigh3);

    redObjectsMask = uint8(redMask1 & greenMask1 & blueMask1);
	redObjectsMask = cast(redObjectsMask, class(redChannel));
    greenObjectsMask = uint8(redMask2 & greenMask2 & blueMask2);
    greenObjectsMask = cast(greenObjectsMask, class(greenChannel));
    blueObjectsMask = uint8(redMask3 & greenMask3 & blueMask3);
    blueObjectsMask = cast(greenObjectsMask, class(blueChannel));


    % smallestAcceptableArea = 100;
    % redObjectsMask = uint8(bwareaopen(redObjectsMask, smallestAcceptableArea));
    % structuringElement = strel('disk', 4);
	% redObjectsMask = imclose(redObjectsMask, structuringElement);
    % redObjectsMask = uint8(imfill(redObjectsMask, 'holes'));
    % 
    % maskedImageR = redObjectsMask .* redChannel;
	% maskedImageG = redObjectsMask .* greenChannel;
	% maskedImageB = redObjectsMask .* blueChannel;

    %maskedRGBImage = cat(3, maskedImageR, maskedImageG, maskedImageB);
end