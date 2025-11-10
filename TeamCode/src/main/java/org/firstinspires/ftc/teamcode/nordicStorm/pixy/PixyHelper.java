package org.firstinspires.ftc.teamcode.nordicStorm.pixy;

import java.util.Deque;
import java.util.Queue;
import java.util.LinkedList;
import java.util.function.ToIntFunction;

public class PixyHelper {

    private int size;
    private Deque<PixyBlock> pixyBlockQueue;

    public PixyHelper(int size) {
        this.size = size;
        this.pixyBlockQueue = new LinkedList<PixyBlock>();
        for (int i = 0; i < 5; i++) {
            this.pixyBlockQueue.add(new EmptyBlock());
        }
    }

    public void update(PixyBlock newBlock) {
        this.pixyBlockQueue.poll();
        this.pixyBlockQueue.add(newBlock);
    }

    public boolean seesBall() {
        int confidence = 0;
        for (PixyBlock block : this.pixyBlockQueue) {
            if (block.isValid())
                confidence++;
        }
        return confidence >= 2;
    }

    private int getWeighted(ToIntFunction<PixyBlock> extractor) {
        if (!seesBall())
            return -1;
        int acc = 0;
        int totalWeight = 0;
        for (PixyBlock block : this.pixyBlockQueue) {
            if (!block.isValid())
                continue;
            int weight = (block == this.pixyBlockQueue.getLast()) ? 2 : 1;
            acc += extractor.applyAsInt(block) * weight;
            totalWeight += weight;
        }
        if (totalWeight == 0)
            return -1;
        return acc / totalWeight;
    }

    public int getMaxArea() {
        if (!seesBall())
            return -1;
        int maxArea = 0;
        for (PixyBlock block : this.pixyBlockQueue) {
            if (!block.isValid())
                continue;
            if (block.height * block.width > maxArea) {
                maxArea = block.height * block.width;
            }
        }
        if (maxArea == 0)
            return -1;
        return maxArea;
    }

    public int getMaxX() {
        if (!seesBall())
            return -1;
        int maxX = 0;
        for (PixyBlock block : this.pixyBlockQueue) {
            if (!block.isValid())
                continue;
            if (block.height * block.width > maxX) {
                maxX = block.centerX;
            }
        }
        if (maxX == 0)
            return -1;
        return maxX;
    }

    public int getWeightedArea() {
        return getWeighted(b -> b.height * b.width);
    }

    public int getWeightedX() {
        return getWeighted(b -> b.centerX);
    }

    public int getWeightedY() {
        return getWeighted(b -> b.centerY);
    }

    public String getData() {
        return"Sees Object:" + seesBall() + " Area: " + getWeightedArea() + " centerX: " + getWeightedX() + " centerY: " + getWeightedY();
    }

}

