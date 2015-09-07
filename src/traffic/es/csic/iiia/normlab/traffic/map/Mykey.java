package es.csic.iiia.normlab.traffic.map;

public class Mykey<A, B, C> {
  private A first;
  private B second;
  private C third;

  public Mykey(A first, B second, C third) {
  	super();
  	this.first = first;
  	this.second = second;
  	this.third=third;
  }

  public int hashCode() {
  	int hashFirst = first != null ? first.hashCode() : 0;
  	int hashSecond = second != null ? second.hashCode() : 0;
  	int hashThird = third != null ? third.hashCode() : 0;

  	return (hashFirst + hashSecond+hashThird) * hashSecond + hashFirst;
  }

  public boolean equals(Object other) {
  	if (other instanceof Mykey) {
  		Mykey otherMykey = (Mykey) other;
  		return 
  		((  this.first == otherMykey.first ||
  			( this.first != null && otherMykey.first != null &&
  			  this.first.equals(otherMykey.first))) &&
  		 (	this.second == otherMykey.second ||
  			( this.second != null && otherMykey.second != null &&
  			  this.second.equals(otherMykey.second))) &&
  		(	this.third == otherMykey.third ||
  			( this.third != null && otherMykey.third != null &&
  			  this.third.equals(otherMykey.third))));
  	}

  	return false;
  }

  public String toString()
  { 
         return "(" + first + ", " + second + ", " + third + ")"; 
  }

  public A getFirst() {
  	return first;
  }

  public void setFirst(A first) {
  	this.first = first;
  }

  public B getSecond() {
  	return second;
  }

  public void setSecond(B second) {
  	this.second = second;
  }
  
  public C getThird() {
  	return third;
  }

  public void setThird(C third) {
  	this.third = third;
  }
}