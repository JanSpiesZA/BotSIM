//Code copied from: http://www.lagers.org.uk/how-to/ht-geom-01/index.html




/**
 * Determine whether a line intersects with a box. <br>
 * The box is represented by the top-left and
 * bottom-right corner coordinates.
 * @param lx0
 * @param ly0
 * @param lx1
 * @param ly1
 * @param rx0
 * @param ry0
 * @param rx1
 * @param ry1
 * @return true if they intersect else false
 */
boolean line_box_xyxy(float lx0, float ly0, float lx1, float ly1, float rx0, float ry0, float rx1, float ry1) {
  int out1, out2;
  float rectWidth = rx1 - rx0;
  float rectHeight = ry1 - ry0;
  if ((out2 = outcode(lx1, ly1, rx0, ry0, rectWidth, rectHeight)) == 0) {
    return true;
  }
  while ( (out1 = outcode (lx0, ly0, rx0, ry0, rectWidth, rectHeight)) != 0) {
    if ((out1 & out2) != 0) {
      return false;
    }
    if ((out1 & (OUT_LEFT | OUT_RIGHT)) != 0) {
      float x = rx0;
      if ((out1 & OUT_RIGHT) != 0) {
        x += rectWidth;
      }
      ly0 = ly0 + (x - lx0) * (ly1 - ly0) / (lx1 - lx0);
      lx0 = x;
    } else {
      float y = ry0;
      if ((out1 & OUT_BOTTOM) != 0) {
        y += rectHeight;
      }
      lx0 = lx0 + (y - ly0) * (lx1 - lx0) / (ly1 - ly0);
      ly0 = y;
    }
  }
  return true;
}

/**
 * Determine whether a line intersects with a box. <br>
 * The box is represented by the top-left corner coordinates
 * and the box width and height.
 * @param lx0
 * @param ly0
 * @param lx1
 * @param ly1
 * @param rx0
 * @param ry0
 * @param rWidth
 * @param rHeight
 * @return true if they intersect else false
 */
boolean line_box_xywh(float lx0, float ly0, float lx1, float ly1, float rx0, float ry0, float rWidth, float rHeight) {
  int out1, out2;
  if ((out2 = outcode(lx1, ly1, rx0, ry0, rWidth, rHeight)) == 0) {
    return true;
  }
  while ( (out1 = outcode (lx0, ly0, rx0, ry0, rWidth, rHeight)) != 0) {
    if ((out1 & out2) != 0) {
      return false;
    }
    if ((out1 & (OUT_LEFT | OUT_RIGHT)) != 0) {
      float x = rx0;
      if ((out1 & OUT_RIGHT) != 0) {
        x += rWidth;
      }
      ly0 = ly0 + (x - lx0) * (ly1 - ly0) / (lx1 - lx0);
      lx0 = x;
    } else {
      float y = ry0;
      if ((out1 & OUT_BOTTOM) != 0) {
        y += rHeight;
      }
      lx0 = lx0 + (y - ly0) * (lx1 - lx0) / (ly1 - ly0);
      ly0 = y;
    }
  }
  return true;
}

/*
 * Used by line - box intersection algorithm
 */
int outcode(float pX, float pY, float rectX, float rectY, float rectWidth, float rectHeight) {
  int out = 0;
  if (rectWidth <= 0) {
    out |= OUT_LEFT | OUT_RIGHT;
  } else if (pX < rectX) {
    out |= OUT_LEFT;
  } else if (pX > rectX + rectWidth) {
    out |= OUT_RIGHT;
  }
  if (rectHeight <= 0) {
    out |= OUT_TOP | OUT_BOTTOM;
  } else if (pY < rectY) {
    out |= OUT_TOP;
  } else if (pY > rectY + rectHeight) {
    out |= OUT_BOTTOM;
  }
  return out;
}

// Used in line-box intersection test
final int OUT_LEFT    = 1;
final int OUT_TOP     = 2;
final int OUT_RIGHT   = 4;
final int OUT_BOTTOM  = 8;