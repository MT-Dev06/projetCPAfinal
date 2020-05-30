package algorithms;

import java.awt.Point;

public class Edge implements Comparable<Edge> {

	private Point a;
	private Point b;
	private double distance;

	public Edge(Point a, Point b) {
		this.a = a;
		this.b = b;
		this.distance = a.distance(b);
	}

	public Edge(Point a, Point b, double distance) {
		this.a = a;
		this.b = b;
		this.distance = distance;
	}

	public Point getA() {
		return a;
	}

	public Point getB() {
		return b;
	}

	public double getDist() {
		return distance;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((a == null) ? 0 : a.hashCode());
		result = prime * result + ((b == null) ? 0 : b.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Edge other = (Edge) obj;
		if (a == null) {
			if (other.a != null)
				return false;
		} else if (!a.equals(other.a))
			return false;
		if (b == null) {
			if (other.b != null)
				return false;
		} else if (!b.equals(other.b))
			return false;
		return true;
	}

	@Override
	public int compareTo(Edge o) {
		int res = 0;
		if (this.distance > o.getDist())
			res = 1;
		else if (this.distance < o.getDist())
			res = -1;
		return res;
	}

	@Override
	public String toString() {
		return "distance=" + distance;
	}
	
	

}
